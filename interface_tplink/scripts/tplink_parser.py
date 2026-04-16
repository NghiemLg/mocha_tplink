#!/usr/bin/env python3

import ast
import yaml
import os
import re
import threading
import pprint
import sys
import pdb

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
import rospkg

# tplinkParser class: Subscribes to tplink radio API data and publishes RSSI (signal strength) 
# values to ROS topics for each robot. It tracks communication quality between robots 
# using their radio MAC addresses and publishes signal strength metrics.
class tplinkParser():
    def __init__(self, this_robot, robot_configs, radio_configs):
        """
        Initialize the tplinkParser with robot and radio configuration.
        
        Args:
            this_robot (str): Name of the current robot running this node
            robot_configs (dict): Dictionary containing all robots' configurations (radios they use, etc.)
            radio_configs (dict): Dictionary containing radio configurations and their associated MAC addresses
        """
        # Check input args
        assert isinstance(this_robot, str)
        assert isinstance(robot_configs, dict)
        assert isinstance(radio_configs, dict)
        
        # MAC_DICT: Main data structure tracking all known radio MAC addresses
        # Each MAC address key maps to: {'rssi': signal strength, 'timestamp': last update time, 
        #                                 'radio': radio name, 'publisher': ROS publisher object}
        self.MAC_DICT = {}

        self.this_robot = this_robot              # Store current robot name
        self.robot_cfg = robot_configs             # Store all robot configurations
        self.radio_cfg = radio_configs             # Store all radio configurations
        self.rate = rospy.Rate(.5)                 # Rate object for timing (0.5 Hz)

        rospy.loginfo(f"{self.this_robot} - tplink API Parser - Starting")

        # Initialize MAC_DICT with all radios from configuration
        # Each MAC address gets default RSSI of -20 (weak signal) and current timestamp
        # Publisher is initially None until assigned to a robot
        for radio in self.radio_cfg.keys():
            for address in self.radio_cfg[radio]['MAC-address']:
                self.MAC_DICT[address] = {}
                self.MAC_DICT[address]['rssi'] = -20                      # Initial RSSI value (weak)
                self.MAC_DICT[address]['timestamp'] = rospy.Time.now()    # Track last update time
                self.MAC_DICT[address]['radio'] = radio                   # Associated radio name
                self.MAC_DICT[address]['publisher'] = None                # ROS publisher (assigned below)

        # Create ROS publishers for each MAC address
        # Only create publishers for MAC addresses whose radio is used by other robots
        # (skip the current robot to avoid self-publishing)
        for mac in self.MAC_DICT.keys():
            for robot in self.robot_cfg.keys():
                # Check if this MAC's radio is used by another robot
                if self.MAC_DICT[mac]['radio'] == self.robot_cfg[robot]['using-radio'] and robot != self.this_robot:
                    # Create publisher: publishes RSSI values to topic 'ddb/tplink/rssi/<robot_name>'
                    self.MAC_DICT[mac]['publisher'] = rospy.Publisher('ddb/tplink/rssi/' + robot, Int32, queue_size = 10)

        # Subscribe to tplink API log messages and enter ROS event loop
        rospy.Subscriber('ddb/tplink/log', String, self.update_dict)
        rospy.spin()  # Continuously listen for messages


    def update_dict(self, data):
        """
        Callback function that processes incoming tplink API log messages.
        Updates RSSI values for known MAC addresses and publishes them to ROS topics.
        
        Args:
            data (String): ROS message containing tplink API data as a string representation of a dictionary
        """
        no_rssi = -1                    # RSSI value when signal is lost/timed out
        dt = rospy.Duration(20.)        # Timeout duration: if no update for 20 seconds, mark as lost

        # Parse incoming tplink API message (string representation of nested dictionary)
        alldata = data.data
        data_dict = ast.literal_eval(data.data)  # Convert string to Python dictionary
        
        # Extract the 'state' from the watchResponse - contains wireless channel and peer data
        state = data_dict['watchResponse']['state']

        # Iterate through all wireless channels and peer information
        # Each channel contains multiple peers (connected radios)
        for wireless_channel in state.keys():
            for wireless_keys in state[wireless_channel].keys():
                # Check if this key is a peer (peer0, peer1, etc.)
                if wireless_keys[0:4] == 'peer':
                    peer = wireless_keys
                    # Case 1: Peer has active RSSI data (connection is active)
                    if 'rssi' in state[wireless_channel][peer].keys():
                        mac = state[wireless_channel][peer]['mac']
                        # Validate MAC address is in our configuration
                        if mac not in self.MAC_DICT.keys():
                            rospy.logerr(f"MAC: {mac} is not in the list of knowns MACs. Is your radio_configs.yaml file correct?")
                            continue
                        
                        # Update RSSI value and timestamp for this MAC address
                        rssi =  state[wireless_channel][peer]['rssi']
                        self.MAC_DICT[mac]['rssi'] = rssi
                        self.MAC_DICT[mac]['timestamp'] = rospy.Time.now()  # Reset timeout counter
                        
                        # Publish RSSI value to ROS topic if this MAC is assigned to a robot
                        if self.MAC_DICT[mac]['publisher'] is not None:
                            self.MAC_DICT[mac]['publisher'].publish(rssi)  # Publish signal strength
                        else:
                            rospy.logwarn(f"{self.this_robot} - tplink API Parser - " +
                                          f"active radio {self.MAC_DICT[mac]['radio']} not assigned to any robot")
                    # Case 2: Peer exists but has no active RSSI (weak/lost signal)
                    elif 'mac' in state[wireless_channel][peer].keys() and 'rssi' not in state[wireless_channel][peer].keys():
                        mac = state[wireless_channel][peer]['mac']
                        # Validate MAC address is in our configuration
                        if mac not in self.MAC_DICT.keys():
                            rospy.logerr(f"MAC: {mac} is not in the list of knowns MACs. Is your radio_configs.yaml file correct?")
                            continue
                        
                        # Check if timeout has occurred (20 second window with no RSSI updates)
                        if rospy.Time.now()-self.MAC_DICT[mac]['timestamp'] > dt:
                            self.MAC_DICT[mac]['rssi'] = no_rssi  # Mark signal as lost
                            
                            # Publish "no signal" value to ROS topic if assigned to a robot
                            if self.MAC_DICT[mac]['publisher'] is not None:
                                self.MAC_DICT[mac]['publisher'].publish(no_rssi)  # Publish lost signal
                            else:
                                rospy.logwarn(f"{self.this_robot} - tplink API Parser - " +
                                              f"active radio {self.MAC_DICT[mac]['radio']} not assigned to any robot")


if __name__ == '__main__':
    """
    Main entry point: Initialize ROS node and load configuration files.
    Launch the tplinkParser to begin processing radio RSSI data.
    """
    rospy.init_node('tplink_listener', anonymous=False)
    
    # Get robot name from ROS launch parameter (default: 'charon' if not specified)
    robot_name = rospy.get_param('~robot_name', 'charon')

    # Load robot configurations from YAML file specified in ROS parameters
    robot_configs_file = rospy.get_param("~robot_configs")
    with open(robot_configs_file, "r") as f:
       robot_configs = yaml.load(f, Loader=yaml.FullLoader)
    
    # Validate that current robot exists in configuration
    if robot_name not in robot_configs.keys():
        rospy.signal_shutdown("Robot not in config file")
        rospy.spin()

    # Load radio configurations from YAML file specified in ROS parameters
    radio_configs_file = rospy.get_param("~radio_configs")
    with open(radio_configs_file, "r") as f:
        radio_configs = yaml.load(f, Loader=yaml.FullLoader)
    
    # Validate that robot's specified radio exists in configuration
    radio = robot_configs[robot_name]["using-radio"]
    if radio not in radio_configs.keys():
        rospy.shutdown("Radio not in config file")
        rospy.spin()

    # Create and start the parser
    tplinkParser(robot_name, robot_configs, radio_configs)
