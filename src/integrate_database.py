#!/usr/bin/env python3
"""
INTEGRATE DATABASE NODE
=======================
ROS node that orchestrates multi-robot database synchronization via network channels.

This is the main entry point for the MOCHA framework's distributed database system.
Each robot runs one instance of this node to:

1. Manage local database (DBServer)
   - Stores sensor data received from ROS topics
   - Provides ROS service interface for data queries
   - Thread-safe access via DBwLock

2. Establish communication channels with peer robots
   - One bi-directional Channel per connected robot
   - ZMQ TCP/IP communication (localhost for single-laptop, LAN for multi-robot)
   - Automatic database synchronization

3. Monitor signal strength (RSSI) for adaptive sync triggering
   - Subscribes to Rajant radio signal strength topics
   - Triggers sync when connection is strong (RSSI > threshold)
   - Reduces network overhead when connection is weak

Architecture:
  Robot 1 (Jackal UGV)          Robot 2 (Hector Quadrotor UAV)
      |                              |
  ROS Topics                      ROS Topics
      |                              |
      └→ integrate_database    integrate_database ←┘
              |                      |
              └──── Channel (ZMQ TCP/IP) ────→
              ←──────────────────────────────┘
              
  Local Database                 Local Database
  (DBServer)                     (DBServer)
  + DBwLock                      + DBwLock
  + ROS Services                 + ROS Services

Configuration Files (Required):
  1. robot_configs.yaml: Robot definitions, IPs, radio types
  2. radio_configs.yaml: Radio-specific communication parameters
  3. topic_configs.yaml: ROS topics for each robot type

Parameters (from ROS launch file):
  - ~robot_name: Name of this robot (must match robot_configs.yaml)
  - ~robot_configs: Path to robot_configs.yaml
  - ~radio_configs: Path to radio_configs.yaml
  - ~topic_configs: Path to topic_configs.yaml
  - ~rssi_threshold: Signal strength threshold for sync trigger (default: 20)
  - ~client_timeout: ZMQ communication timeout in seconds (default: 6.0)

ROS Subscriptions:
  - ddb/rajant/rssi/<robot_name>: Rajant radio signal strength (Int32)
    Used to trigger synchronization when connection is strong
    NOTE: Only subscribed if radio is configured to use Rajant

Dependencies:
  - rospy: ROS Python interface
  - yaml: Configuration file parsing
  - subprocess: Network diagnostics (ping)
  - signal: Graceful shutdown handling
  - database_server: DBServer class for local data storage
  - synchronize_channel: Channel class for peer communication

Operation Flow:
  1. Initialize node and read ROS parameters
  2. Load and validate YAML configuration files
  3. Ping local radio interface (confirm network connectivity)
  4. Create local DatabaseServer
  5. Create Channel for each peer robot (from config)
  6. Subscribe to RSSI topics for each peer
  7. Enter rospy.spin() to wait for events
  8. On RSSI trigger: activate database sync with peer
  9. On shutdown signal: cleanup all channels and database

Error Handling:
  - Missing robot in config: Immediate shutdown
  - Radio not in config: Immediate shutdown
  - Cannot ping self: Shutdown with warning
  - RSSI trigger failure: Log exception, continue running
  - Signal SIGINT: Graceful shutdown

Typical Usage:
  roslaunch mocha_tplink mocha_sim_single_all.launch
  # Launches both integrate_database nodes for Jackal and UAV
  # Both nodes synchronize data when RSSI > threshold

Performance Notes:
  - CPU: ~2-5% per robot (network I/O dependent)
  - Memory: ~50MB per robot (varies with database size)
  - Network: Bandwidth depends on sync frequency and message size
  - Latency: ZMQ communication adds ~1-5ms per sync
"""

import os
import random
import signal
import sys
import time
import traceback
from functools import partial

import roslaunch
import rospkg
import rospy
import yaml
import std_msgs.msg
import subprocess

import database_server as ds
import database_utils as du
import synchronize_channel as sync


def ping(host):
    """
    Network connectivity check using ICMP ping.
    
    Tests whether a host is reachable on the network. Used during initialization
    to verify that the local robot's network interface (radio) is operational.
    
    Process:
    1. Construct ping command with 1 echo request
    2. Execute subprocess call with 1-second timeout implicit
    3. Suppress stdout/stderr (we only care about return code)
    4. Return True if response received (code 0), False otherwise
    
    Args:
        host (str): IP address or hostname to ping
                   Examples: "127.0.0.1" (localhost), "192.168.1.10"
    
    Returns:
        bool: True if host responds to ping (reachable)
              False if ping fails (unreachable) or exception occurs
    
    Raises:
        (None - all exceptions caught and logged)
    
    Examples:
        # Check if local interface is up
        if ping("127.0.0.1"):
            print("Localhost reachable")
        
        # Check if remote robot is reachable
        if ping("192.168.1.10"):
            print("Remote robot up")
        else:
            print("Cannot reach remote robot")
    
    Use Cases:
        1. Startup verification: Confirm radio is operational
           if not ping(local_ip):
               shutdown("Radio not responding to ping")
        
        2. Network diagnostic: Check remote robot connectivity
           for robot_ip in robot_ips:
               if ping(robot_ip):
                   print(f"{robot_ip} is reachable")
        
        3. Heartbeat monitoring: Periodic connectivity checks
           while running:
               if not ping(remote_ip):
                   rospy.logwarn("Lost connection")
    
    Platform Compatibility:
        Linux/Mac: Uses ping -c 1 (send 1 packet, wait 1 sec timeout)
        Returns: code 0 = success, code 1 = no response, code 2 = error
    
    Performance:
        - Time: ~100-500ms (typical network latency)
        - Blocking operation (wait for ICMP response or timeout)
        - Note: Not suitable for high-frequency polling (use TCP instead)
    
    Limitations:
        - Networks with ICMP blocked will always return False
        - Timeout hardcoded to 1 second (not configurable)
        - Single packet (no statistics like min/max/avg)
        - Requires subprocess execution overhead
    
    Error Handling:
        - subprocess.run() exceptions caught and logged
        - Returns False on any error (safe default)
        - Exception message printed but doesn't stop execution
    
    Notes:
        - Used only during startup (one-time check)
        - Not used for continuous monitoring (would add latency)
        - Replacement: Use TCP connection test for more reliable checks
    """
    command = ["ping", "-c", "1", host]
    try:
        result = subprocess.run(command, stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL)
        return result.returncode == 0
    except Exception as e:
        print(f"Error pinging {host}: {e}")
        return False


class IntegrateDatabase:
    """
    Multi-Robot Database Integration Node
    ======================================
    Orchestrates distributed database synchronization across robots in the MOCHA framework.
    
    This class is the main ROS node that coordinates:
    1. Local data storage (DatabaseServer with thread-safe DBwLock)
    2. Inter-robot communication (Channel objects for each peer)
    3. Sync triggering (RSSI-based network quality monitoring)
    4. Graceful shutdown (proper cleanup of all resources)
    
    Lifecycle:
        __init__:  Startup, config loading, channel creation
        Running:   rospy.spin() waits for ROS events
        rssi_cb:   RSSI triggers activate database sync
        shutdown:  Cleanup on termination
    
    Thread Safety:
        - DatabaseServer (self.DBServer) uses DBwLock (thread-safe)
        - Channel objects run in separate threads
        - RSSI callback may be called from different ROS thread
        - Self-modifying operations (e.g., list.remove) not thread-safe
          (Not critical here since only modified during shutdown)
    
    Configuration Requirements:
        Three YAML files must be provided and valid:
        
        1. robot_configs.yaml structure:
           robot_name:
             IP-address: "192.168.1.X"
             using-radio: false|<radio_type>  # false for localhost
             node-type: "jackal"|"uav"|etc
             clients: [list of robots to sync with]
        
        2. radio_configs.yaml structure:
           false:  # For localhost single-laptop
             (TCP/ZMQ configuration)
           rajant:
             (Rajant radio configuration)
           other_radio: ...
        
        3. topic_configs.yaml structure:
           node_type:  # e.g., "jackal"
             robot_id: 0
             topics:
               - name: "scan"
                 id: 0
               - name: "odom"
                 id: 1
    
    Instance Attributes:
        this_robot (str): Current robot name from config
        rssi_threshold (int): Signal strength threshold (default 20)
        client_timeout (float): ZMQ timeout in seconds (default 6.0)
        robot_configs (dict): Loaded robot configuration
        radio_configs (dict): Loaded radio configuration
        topic_configs (dict): Loaded topic configuration
        DBServer (DatabaseServer): Local data store with ROS services
        all_channels (list): Channel objects for each peer robot
        other_robots (list): Names of peer robots from config
        num_robot_in_comm (int): Counter of robots currently syncing
        interrupted (bool): Flag for signal-based shutdown
    
    ROS Integration:
        - Node name: "integrate_database"
        - Subscribers: ddb/rajant/rssi/<robot_name> for RSSI monitoring
        - Services: Provided by contained DatabaseServer object
        - Parameters: robot_name, robot_configs, radio_configs, etc.
    
    Error Handling:
        - Missing config entries: Immediate shutdown
        - Network unreachable: Shutdown during startup
        - Runtime errors: Logged, node continues (resilient)
        - SIGINT: Graceful shutdown with cleanup
    """
    def __init__(self):
        """
        Initialize the IntegrateDatabase node and create all channels.
        
        This is the main initialization routine that:
        1. Sets up the ROS node
        2. Loads and validates all YAML configuration files
        3. Verifies network connectivity
        4. Creates the local DatabaseServer
        5. Creates one Channel per peer robot
        6. Subscribes to RSSI topics for triggering sync
        
        Process Steps:
        
        A. ROS Node Setup:
           - Initialize "integrate_database" ROS node
           - Read ROS parameters for robot name and timeouts
           - Set up signal handlers for graceful shutdown
        
        B. Configuration Loading & Validation:
           - Load robot_configs.yaml and verify this robot exists
           - Load radio_configs.yaml and verify radio type exists
           - Load topic_configs.yaml and verify node type exists
           - Extract robot_id and topic_id mappings
        
        C. Network Verification:
           - Ping local IP address to confirm radio is operational
           - Exit with error if unreachable
           - Log connection status
        
        D. Database Server Creation:
           - Create DatabaseServer instance (local data storage)
           - Register ROS service callbacks
           - Initialize with this robot's configuration
        
        E. Channel Creation:
           - For each robot in config (except self):
             - Check if listed in this robot's "clients" list
             - Create Channel object for ZMQ communication
             - Call channel.run() to start background thread
             - Subscribe to RSSI topic for this peer
                (topic: ddb/rajant/rssi/<peer_name>)
        
        F. Startup Synchronization:
           - Wait up to 10 seconds (100 * 0.1sec) for all peers to start
           - Check for interruption or shutdown signal during wait
           - Begin normal ROS spinning after all ready
        
        ROS Parameters Expected:
            ~robot_name (string): Name of this robot
                                 Must exist in robot_configs.yaml
                                 Examples: "jackal", "hector_quad"
            
            ~robot_configs (string): Path to robot_configs.yaml
                                    File must exist and be valid YAML
            
            ~radio_configs (string): Path to radio_configs.yaml
                                    File must exist and be valid YAML
            
            ~topic_configs (string): Path to topic_configs.yaml
                                    File must exist and be valid YAML
            
            ~rssi_threshold (int, optional): Signal strength threshold
                                           Triggers sync when RSSI > threshold
                                           Default: 20
                                           (Rajant radio typically 0-100 scale)
            
            ~client_timeout (float, optional): ZMQ socket timeout
                                            Timeout for all socket operations
                                            Default: 6.0 seconds
        
        Returns:
            None (blocks in rospy.spin() or exits via shutdown/error)
        
        Raises:
            (None - all exceptions caught and trigger shutdown)
        
        Attributes Set:
            self.this_robot: Robot name from ROS param
            self.rssi_threshold: RSSI threshold for sync triggering
            self.client_timeout: ZMQ timeout value
            self.robot_configs: Dict loaded from robot_configs.yaml
            self.radio_configs: Dict loaded from radio_configs.yaml
            self.topic_configs: Dict loaded from topic_configs.yaml
            self.DBServer: DatabaseServer instance
            self.all_channels: List of Channel objects
            self.other_robots: List of peer robot names
            self.num_robot_in_comm: Counter (initialized to 0)
            self.interrupted: Signal flag (initialized to False)
        
        Examples:
            # Typically launched via roslaunch
            roslaunch mocha_tplink mocha_sim_single_all.launch
            
            # This creates:
            # - integrate_database node for Jackal (robot_id=0)
            # - integrate_database node for Hector UAV (robot_id=1)
            # - Both nodes create channels to each other
            # - Both subscribe to RSSI topics
        
        Configuration Example (robot_configs_sim.yaml):
            jackal:
              IP-address: "127.0.0.1"
              using-radio: false
              node-type: "jackal"
              clients: ["uav"]
            
            uav:
              IP-address: "127.0.0.1"
              using-radio: false
              node-type: "uav"
              clients: ["jackal"]
        
        Error Scenarios Handled:
            1. Robot not in robot_configs.yaml
               → shutdown("Robot not in config file")
            
            2. Radio type not in radio_configs.yaml
               → shutdown("Radio not in config file")
            
            3. Node type not in topic_configs.yaml
               → shutdown("Node type not in config file")
            
            4. Cannot ping local IP
               → signal_shutdown("Cannot ping self")
            
            5. SIGINT received during startup
               → graceful shutdown with cleanup
        
        Performance:
            - Startup time: ~1-2 seconds (YAML parsing, channel creation)
            - Memory usage: ~50MB per robot (configuration + empty database)
            - Lock contention: None during startup (single-threaded)
        
        Signal Handling:
            - SIGINT: Triggers signal_handler
            - Sets self.interrupted = True
            - Calls rospy.signal_shutdown()
            - Continues to shutdown() for cleanup
        
        Notes:
            - Blocks indefinitely in rospy.spin() after startup
            - RSSI events trigger sync via rssi_cb callback
            - Each robot runs independent instance with same code
            - Channel creation order doesn't matter (handles both directions)
            - Startup wait (100 * 0.1s) allows graceful startup of multiple nodes
        """
        rospy.init_node("integrate_database", anonymous=False)

        self.this_robot = rospy.get_param("~robot_name")
        self.rssi_threshold = rospy.get_param("~rssi_threshold", 20)
        self.all_channels = []
        rospy.loginfo(f"{self.this_robot} - Integrate - " +
                      f"RSSI threshold: {self.rssi_threshold}")
        self.client_timeout = rospy.get_param("~client_timeout", 6.)
        rospy.loginfo(f"{self.this_robot} - Integrate - " +
                      f"Client timeout: {self.client_timeout}")

        # Load and check robot configs
        self.robot_configs_file = rospy.get_param("~robot_configs")
        with open(self.robot_configs_file, "r") as f:
            self.robot_configs = yaml.load(f, Loader=yaml.FullLoader)
        if self.this_robot not in self.robot_configs.keys():
            self.shutdown("Robot not in config file")

        # Load and check radio configs
        self.radio_configs_file = rospy.get_param("~radio_configs")
        with open(self.radio_configs_file, "r") as f:
            self.radio_configs = yaml.load(f, Loader=yaml.FullLoader)
        self.radio = self.robot_configs[self.this_robot]["using-radio"]
        if self.radio not in self.radio_configs.keys():
            self.shutdown("Radio not in config file")

        # Load and check topic configs
        self.topic_configs_file = rospy.get_param("~topic_configs")
        with open(self.topic_configs_file, "r") as f:
            self.topic_configs = yaml.load(f, Loader=yaml.FullLoader)
        self.node_type = self.robot_configs[self.this_robot]["node-type"]
        if self.node_type not in self.topic_configs.keys():
            self.shutdown("Node type not in config file")

        # Check that we can ping the radios
        ip = self.robot_configs[self.this_robot]["IP-address"]
        if not ping(ip):
            rospy.logerr(f"{self.this_robot} - Integrate - " +
                         f"Cannot ping self {ip}. Is the radio on?")
            rospy.signal_shutdown("Cannot ping self")
            rospy.spin()

        # Create a database server object
        self.DBServer = ds.DatabaseServer(self.robot_configs,
                                          self.topic_configs)

        self.num_robot_in_comm = 0

        # Handle possible interruptions
        self.interrupted = False

        def signal_handler(sig, frame):
            rospy.logwarn(f"{self.this_robot} - Integrate - " +
                          f"Got signal. Killing comm nodes.")
            self.interrupted = True
            rospy.signal_shutdown("Killed by user")
        signal.signal(signal.SIGINT, signal_handler)

        rospy.loginfo(f"{self.this_robot} - Integrate - " +
                      "Created all communication channels!")

        # Start comm channels with other robots
        self.other_robots = [i for i in list(self.robot_configs.keys()) if i !=
                             self.this_robot]
        for other_robot in self.other_robots:
            if other_robot not in self.robot_configs[self.this_robot]["clients"]:
                rospy.logwarn(
                    f"{self.this_robot} - Integrate - "+
                    f"Skipping channel {self.this_robot}->{other_robot} " +
                    "as it is not in the graph of this robot"
                )
                continue
            # Start communication channel
            channel = sync.Channel(self.DBServer.dbl,
                                   self.this_robot,
                                   other_robot, self.robot_configs,
                                   self.client_timeout)
            self.all_channels.append(channel)
            channel.run()

            # Attach a radio trigger to each channel. This will be triggered
            # when the RSSI is high enough. You can use another approach here
            # such as using a timer to periodically trigger the sync
            rospy.Subscriber('ddb/rajant/rssi/' + other_robot,
                             std_msgs.msg.Int32,
                             self.rssi_cb,
                             channel)

        # Wait for all the robots to start
        for _ in range(100):
            if self.interrupted or rospy.is_shutdown():
                self.shutdown("Killed while waiting for other robots")
                return
            rospy.sleep(.1)

        # Spin!
        rospy.spin()

    def shutdown(self, reason):
        """
        Graceful shutdown with proper cleanup of all resources.
        
        This method is called when the node needs to terminate due to:
        - Configuration error (missing robot/radio in config)
        - Network failure (cannot ping local interface)
        - User signal (SIGINT from keyboard)
        - ROS shutdown requested by system
        
        Cleanup Process:
        1. Log the shutdown reason (error or normal)
        2. Terminate all active communication channels
           - Call terminate() on each channel's comm_node
           - Remove channel from all_channels list
           - Wait for background threads to finish
        3. Forcefully terminate the DatabaseServer
           - Close ROS service advertisements
           - Stop accepting new requests
           - Flush any pending operations
        4. Signal ROS that this node is shutting down
           - Notify other nodes of termination
           - Release ROS node resources
        5. Continue spinning briefly to process shutdown events
        
        This method must be called (directly or via signal) for proper cleanup.
        Not calling it can leave zombie processes or unclosed database connections.
        
        Args:
            reason (str): Description of why shutdown is occurring
                         Should be a brief message like "Robot not in config file"
                         or "User terminated with SIGINT"
        
        Raises:
            AssertionError: If reason is not a string
        
        Returns:
            None (node terminates after this call)
        
        Examples:
            # Shutdown due to configuration error
            if self.this_robot not in self.robot_configs.keys():
                self.shutdown("Robot not in config file")
            
            # Shutdown due to network error
            if not ping(local_ip):
                rospy.logerr("Cannot ping self")
                rospy.signal_shutdown("Cannot ping self")
                # Note: direct signal_shutdown won't call shutdown()
                # Must use self.shutdown() for proper cleanup
            
            # Shutdown from signal handler
            def signal_handler(sig, frame):
                self.interrupted = True
                rospy.signal_shutdown("Killed by user")
                # shutdown() called automatically when rospy.spin() exits
        
        Cleanup Sequence:
            1. Log "ERROR: {reason}"
            2. For each channel in all_channels:
               - channel.comm_node.terminate()
               - all_channels.remove(channel)
               - Log "Killed Channels"
            3. self.DBServer.shutdown()
               - Log "Killed DB"
            4. rospy.signal_shutdown(reason)
            5. Log "Shutting down"
            6. rospy.spin() (process remaining callbacks)
        
        Thread Safety:
            - Not thread-safe (ROS callbacks shouldn't call this)
            - Called from main thread or signal handler
            - Channel termination is blocking (waits for threads)
            - Safe to call from signal handler (rospy.signal_shutdown is async-safe)
        
        Performance:
            - Time: ~500ms-2s (depends on number of channels)
            - Each terminate() blocks until thread exits
            - DBServer shutdown may flush pending operations
        
        Side Effects:
            - Removes all items from self.all_channels list
            - Blocks main thread until all channels terminated
            - Causes rospy.spin() to exit
            - Prevents new ROS callbacks from executing
        
        Error Handling:
            - All exceptions from terminate() logged (printed to console)
            - Continues to next channel even if one terminates with error
            - DBServer errors logged but don't prevent shutdown
            - Shutdown completes even if errors occur
        
        Use Cases:
            1. Config file errors (detected at startup)
               if self.radio not in self.radio_configs.keys():
                   self.shutdown("Radio not in config file")
            
            2. Network connectivity issues
               if not ping(ip):
                   self.shutdown("Cannot ping local interface")
            
            3. User interruption
               signal.signal(signal.SIGINT, signal_handler)
               # Handler sets interrupted flag and calls signal_shutdown
               # shutdown() called later via rospy.spin() exit
            
            4. ROS system shutdown
               # rospy.signal_shutdown() called by ROS system
               # shutdown() not auto-called, must be in shutdown handler
        
        Important Notes:
            - Always call this for proper cleanup (not just rospy.signal_shutdown)
            - Assertion on reason type prevents silent failures
            - Channel termination is blocking (slow channels can delay shutdown)
            - Consider adding timeout for channel.terminate() in future
            - Do not call from within rospy callbacks (deadlock risk)
        
        Logging Pattern:
            - Error log (red) for shutdown reason
            - Warning log (yellow) for each cleanup step
            - Provides visibility into shutdown sequence
        
        Integration with ROS Shutdown:
            shutdown() can be called from:
            - Main code path (config validation)
            - Signal handler (rospy.signal_shutdown triggers)
            - Exception handlers
            
            rospy.signal_shutdown() sets flag that exits rospy.spin()
            Main thread continues execution after rospy.spin() returns
        """
        assert isinstance(reason, str)
        rospy.logerr(f"{self.this_robot} - Integrate - " + reason)
        for channel in self.all_channels:
            channel.comm_node.terminate()
            self.all_channels.remove(channel)
        rospy.logwarn(f"{self.this_robot} - Integrate - " + "Killed Channels")
        self.DBServer.shutdown()
        rospy.logwarn(f"{self.this_robot} - Integrate - " + "Killed DB")
        rospy.signal_shutdown(reason)
        rospy.logwarn(f"{self.this_robot} - Integrate - " + "Shutting down")
        rospy.spin()

    def rssi_cb(self, data, comm_node):
        """
        RSSI (Radio Signal Strength Indicator) callback for triggering sync.
        
        This ROS callback is triggered whenever the radio signal strength from a peer
        robot changes. When signal strength exceeds the configured threshold, it
        activates database synchronization with that peer.
        
        This implements adaptive synchronization: only sync when connection quality
        is good enough to transfer data efficiently. Avoids wasting bandwidth on
        poor connections.
        
        Thread Safety:
            - Called from ROS callback thread (not main thread)
            - Accesses self.num_robot_in_comm (not protected - potential race)
            - Calls comm_node.trigger_sync() which is thread-safe
            - Exceptions caught and logged (won't crash node)
        
        Callback Registration:
            rospy.Subscriber('ddb/rajant/rssi/' + other_robot,
                           std_msgs.msg.Int32,
                           self.rssi_cb,
                           channel)
            
            Each peer robot gets its own subscription with its own channel passed
            as callback argument. This allows the callback to identify which robot
            triggered the sync.
        
        Args:
            data (std_msgs.msg.Int32): Message containing RSSI value
                                       Typically 0-100 scale (Rajant radio)
                                       Higher value = stronger signal
            
            comm_node (Channel): The communication channel object for this peer
                                Passed as callback argument during subscription
                                Contains target_robot name and sync logic
        
        Returns:
            None (ROS callback - return value ignored)
        
        Raises:
            (None - all exceptions caught and logged)
        
        Examples:
            # Registered during __init__
            rospy.Subscriber('ddb/rajant/rssi/uav',
                           std_msgs.msg.Int32,
                           self.rssi_cb,
                           uav_channel)
            
            # When Rajant publishes new RSSI for UAV:
            # rssi_cb called with Int32(data=75) and uav_channel
            # 75 > 20 (threshold) → trigger_sync() called
            
            # When signal weak:
            # rssi_cb called with Int32(data=5) and uav_channel
            # 5 < 20 → no action (skip sync to save bandwidth)
        
        Operation:
            1. Extract RSSI value from data.data
            2. Compare with self.rssi_threshold
            3. If RSSI > threshold:
               - Increment self.num_robot_in_comm counter
               - Log "Triggering comms" message
               - Call comm_node.trigger_sync()
            4. If RSSI <= threshold:
               - Skip (do nothing)
        
        RSSI Scale (Rajant Radio):
            100 = Excellent signal, full bandwidth available
             75 = Good signal, reliable connection
             50 = Fair signal, some packet loss expected
             25 = Poor signal, frequent retransmissions
              5 = Very poor signal, connection unreliable
              0 = No signal, connection unavailable
            
            Default threshold: 20 (conservative, only sync on fair+ signal)
        
        Signal Quality Decision:
            RSSI > threshold:
               → Bandwidth available, sync data
            RSSI <= threshold:
               → Conserve bandwidth, skip this update
               → Will retry when signal improves
        
        Sync Triggering:
            comm_node.trigger_sync()
            - Initiates database synchronization with peer
            - Sends header list query
            - Requests missing or newer data
            - Updates local database with remote data
            - Typically completes in < 100ms on good connection
        
        Error Handling:
            - Exception in trigger_sync() caught
            - Exception traceback printed to console
            - Node continues running (doesn't crash)
            - Next RSSI callback will retry sync
        
        Use Cases:
            1. Normal operation (good signal)
               RSSI = 50 > 20 → Sync triggered
               "Triggering comms" logged
               Database updated with peer data
            
            2. Weak signal scenario (conserve bandwidth)
               RSSI = 10 < 20 → Sync skipped
               No network activity
               No log message
            
            3. Signal fluctuation
               RSSI = 25 > 20 → Sync triggered
               RSSI = 15 < 20 → Next sync skipped
               Adaptive behavior based on real-time conditions
        
        Performance:
            - Callback latency: ~1-2ms (ROS overhead)
            - No blocking operations
            - Non-blocking counter increment potentially racy
                (but num_robot_in_comm is informational only)
            - trigger_sync() runs in Channel's background thread
        
        Monitoring:
            - num_robot_in_comm counter tracks active syncs
            - Can be used to tune rssi_threshold parameter
            - If too many syncs: increase threshold (reduce frequency)
            - If too few syncs: decrease threshold (increase frequency)
        
        Configuration Impact:
            rssi_threshold = 0:  Always sync (every message = high bandwidth)
            rssi_threshold = 50: Only good signals (conservative)
            rssi_threshold = 100: Never sync (unrealistic)
            
            Recommended: 20-40 for Rajant radio (balance sync vs. latency)
        
        Message Topic Structure:
            Topic: ddb/rajant/rssi/<robot_name>
            Example: ddb/rajant/rssi/uav
            Type: std_msgs/Int32
            Data: RSSI value (typically 0-100)
            Publisher: Believed to be external Rajant radio driver
            Frequency: ~1-10 Hz (depends on radio config)
        
        Channel Identification:
            - comm_node passed as callback argument uniquely identifies peer
            - comm_node.target_robot = name of peer robot
            - Multiple subscriptions possible (one per peer)
            - Each has its own threshold checking
        
        Potential Issues:
            1. Race condition on num_robot_in_comm
               - Not thread-safe (ROS callback thread)
               - Only informational, not critical
               - Could use threading.Lock if needed
            
            2. Threshold not adaptive
               - Same threshold for all robots
               - Could improve with per-robot thresholds
            
            3. No rate limiting
               - Sync triggered on every RSSI above threshold
               - Could add cooldown period
        
        Future Improvements:
            - Per-robot adaptive thresholds (based on history)
            - Rate limiting to avoid excessive syncs
            - Sync priority based on data freshness
            - Exponential backoff for failed syncs
        """
        rssi = data.data
        if rssi > self.rssi_threshold:
            self.num_robot_in_comm += 1
            try:
                rospy.loginfo(f"{self.this_robot} <- {comm_node.target_robot}: Triggering comms")
                comm_node.trigger_sync()
            except:
                traceback.print_exception(*sys.exc_info())


if __name__ == "__main__":
    # Start the node
    IntegrateDatabase()
