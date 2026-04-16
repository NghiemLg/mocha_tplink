#!/usr/bin/env python3
import sys
import subprocess
from threading  import Thread
from queue import Queue, Empty
from pprint import pprint
import sys
import os
import time
import yaml
import re
import pdb
import string
import hashlib
import random
import rospy
import std_msgs.msg
import rospkg

# tplinkQuery: Queries a tplink radio device via Java API and publishes wireless peer data
# This script runs a Java jar process to communicate with the tplink radio, parses its output,
# and publishes structured data to ROS topics for other nodes to consume.

def randomNumber(stringLength=4):
    """Generate a random 4-digit number as a string.
    Used to create unique keys during YAML parsing to avoid duplicate key overwrites.
    """
    number = random.randint(1000, 9999)
    return str(number)


def enqueue_output(out, queue):
    """Save subprocess output to a queue for asynchronous processing.
    
    Continuously reads lines from a subprocess pipe and puts them into a queue.
    This allows the main thread to read process output without blocking.
    
    Args:
        out: File object (stdout pipe) from subprocess
        queue: Queue.Queue object to store output lines
    """
    for line in iter(out.readline, b''):
        queue.put(line)  # Put each line into the queue
    out.close()         # Close the pipe when done


def ping_ip(ip_address):
    """Check if an IP address is reachable via ping.
    
    Args:
        ip_address (str): IP address to ping
    
    Returns:
        bool: True if ping succeeds, False otherwise
    """
    try:
        # Run ping with single packet (-c 1) and 1 second timeout (-W 1)
        result = subprocess.run(["ping", "-c", "1", "-W", "1", ip_address],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True)
        return result.returncode == 0
    except subprocess.CalledProcessError:
        # Ping failed or timed out
        return False


def line_parser(line_bytes):
    """Parse tplink radio API output and convert to YAML-compatible format.
    
    tplink output uses a JSON-like format that's not valid YAML. This function
    transforms it into valid YAML by:
    - Replacing { with : (start of key-value pairs)
    - Removing } (end of objects)
    - Adding random suffixes to 'wireless' and 'peer' keys to avoid duplicates
    - Fixing MAC address formatting
    
    Args:
        line_bytes (bytes): Raw output line from tplink Java process
    
    Returns:
        str: Line formatted as valid YAML
    """
    line_str = line_bytes.decode('unicode-escape')
    line_str = line_str.replace("{", ":")
    line_str = line_str.replace("}", "")
    # Add random numbers to wireless/peer keys to create unique YAML keys
    # (prevents YAML from overwriting duplicate keys)
    line_str = re.sub("wireless",
                      "wireless-" + randomNumber(), line_str)
    line_str = re.sub("peer",
                      "peer-" + randomNumber(), line_str)
    # MAC addresses need special handling due to previous transformations
    if line_str.replace(" ", "")[:4] == "mac:":
        separator = line_str.find(":") + 2
        mac_str = line_str[separator:]
        # Previous replacements may have modified the MAC; restore it properly
        mac_str = mac_str.replace(":", "{")
        mac_bytes = bytes(mac_str, 'raw_unicode_escape')
        # Format MAC address as hex pairs separated by colons (e.g., "aa:bb:cc:dd:ee:ff")
        mac_decoded = ":".join(["%02x" % c for c in mac_bytes[1:-2]])
        line_str = line_str[:separator] + mac_decoded + "\n"
    return line_str


ON_POSIX = 'posix' in sys.builtin_module_names

if __name__ == "__main__":
    """
    Main entry point: Initialize ROS node, load configurations, start Java radio query process,
    and continuously publish tplink radio data to ROS topics.
    """
    rospy.init_node("tplink_query", anonymous=False)

    # Get robot name from ROS parameter (default: 'charon' if not specified)
    robot_name = rospy.get_param('~robot_name', 'charon')

    # Load robot configurations from YAML file
    robot_configs_file = rospy.get_param("~robot_configs")
    with open(robot_configs_file, "r") as f:
       robot_configs = yaml.load(f, Loader=yaml.FullLoader)
    
    # Validate that current robot exists in configuration
    if robot_name not in robot_configs.keys():
        rospy.signal_shutdown("Robot not in config file")
        rospy.spin()

    # Load radio configurations from YAML file
    radio_configs_file = rospy.get_param("~radio_configs")
    with open(radio_configs_file, "r") as f:
        radio_configs = yaml.load(f, Loader=yaml.FullLoader)
    
    # Extract and validate the radio assigned to this robot
    radio = robot_configs[robot_name]["using-radio"]
    if radio not in radio_configs.keys():
        rospy.shutdown("Radio not in config file")
        rospy.spin()

    # Get the radio name and its IP address from configuration
    tplink_name = robot_configs[robot_name]['using-radio']
    if tplink_name in radio_configs.keys():
        target_ip = radio_configs[tplink_name]['computed-IP-address']
    else:
        rospy.logerr(f"Radio {tplink_name} for robot {robot_name} not found in configs")
        rospy.signal_shutdown("Radio not in configs")
        rospy.spin()


    # Create ROS publisher for tplink radio data
    # Publishes parsed radio state to 'ddb/tplink/log' topic
    pub = rospy.Publisher('ddb/tplink/log', std_msgs.msg.String, queue_size=10)

    # Locate the tplink Java jar file within the ROS package
    rospack = rospkg.RosPack()
    ros_path = rospack.get_path('interface_tplink')

    # Path to the tplink Java jar executable
    java_bin = os.path.join(ros_path, 'scripts',
                            'thirdParty/watchstate/bcapi-watchstate-11.19.0-SNAPSHOT-jar-with-dependencies.jar')

    # Start Java subprocess to query the tplink radio
    # The jar takes the radio IP address as a command-line argument
    p = subprocess.Popen(['java',
               '-jar',
               java_bin,
               target_ip], stdout=subprocess.PIPE, close_fds=ON_POSIX)
    
    # Create queue and daemon thread to asynchronously read subprocess output
    q = Queue()
    t = Thread(target=enqueue_output, args=(p.stdout, q))
    t.daemon = True  # Thread dies with the program
    t.start()

    # Log startup and verify radio connectivity
    rospy.loginfo(f"{robot_name} - tplink API Query - Starting on {tplink_name}")

    # Ping the radio to verify it's reachable
    if ping_ip(target_ip):
        rospy.loginfo(f"{robot_name} - tplink API Query - ping success")
    else:
        rospy.logerr(f"{robot_name} - tplink API Query - tplink ping failed")
        rospy.signal_shutdown("tplink IP")
        rospy.spin()

    # Main loop: continuously process radio data and publish to ROS
    while not rospy.is_shutdown():
        # Monitor Java subprocess health - restart if it crashes
        if not t.is_alive():
            rospy.logerr(f'{robot_name}: tplink Java process died! Restarting...')
            # Restart the Java process
            p = subprocess.Popen(['java',
                       '-jar',
                       java_bin,
                       target_ip], stdout=subprocess.PIPE, close_fds=ON_POSIX)
            q = Queue()
            t = Thread(target=enqueue_output, args=(p.stdout, q))
            t.daemon = True  # Thread dies with the program
            t.start()

        # Sleep briefly to give Java process time to generate output (DO NOT REMOVE)
        rospy.sleep(1)

        try:
            # Try to get one line of output from the queue (non-blocking)
            line = q.get_nowait()
        except Empty:
            # No output available yet, continue loop
            pass
        else:
            # Got at least one line; parse it
            answ_array = line_parser(line)
            # Continue draining the queue for additional lines
            while not rospy.is_shutdown():
                try:
                    newline = q.get_nowait()
                except Empty:
                    break
                else:
                    # Accumulate parsed lines
                    answ_array += line_parser(newline)
            
            # Convert and publish the parsed data
            try:
                # Load parsed data as YAML dictionary
                yaml_res = yaml.load(answ_array, Loader=yaml.Loader)
                # Only publish if result is a dictionary (valid tplink response)
                if type(yaml_res) == type({}):
                    # Publish the parsed data as a string
                    pub.publish(str(yaml_res))
                else:
                    rospy.logerr(f"{robot_name}: YAML from tplink did not look like an object!")
            except yaml.scanner.ScannerError:
                rospy.logerr(f"{robot_name}: Could not parse YAML from tplink!")
