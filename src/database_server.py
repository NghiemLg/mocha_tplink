#!/usr/bin/env python3
"""
DATABASE SERVER MODULE
======================
This module implements the DatabaseServer class, which manages a thread-safe database
for storing and retrieving multi-robot sensor data and messages in the MOCHA (Multi-Robot
Operational Coordination and Heterogeneous Autonomy) framework.

The DatabaseServer:
1. Maintains a centralized database of messages from all robots
2. Provides ROS services for adding, updating, querying, and retrieving messages
3. Handles thread-safe database operations using locks
4. Manages message priorities and timestamps
5. Filters messages by robot ID, topic ID, and other criteria

This server runs on each robot and stores data that can be shared across the multi-robot
network via ZMQ communication channels.

Dependencies:
- rospy: ROS Python interface
- mocha_core.srv: Custom ROS service definitions
- database: Custom database module with thread safety
- database_utils: Utility functions for database operations
"""

import os
import threading
import rospy
import mocha_tplink.srv
import database
import pdb
import database_utils as du


class DatabaseServer:
    """ Starts a clean database and offers an API-like service
    to interact with the database.

    This class creates a thread-safe database instance and exposes ROS services
    that other nodes can call to:
    - Add or update messages in the database (AddUpdateDB service)
    - Retrieve message content by header/hash (GetDataHeaderDB service)
    - Query/select messages by robot_id and topic_id (SelectDB service)

    The database stores messages from multiple robots and topics, allowing
    MOCHA nodes to share sensor data and maintain consistency across the
    multi-robot network.

    Please see the list of services in the srv folder:
    - mocha_core/srv/AddUpdateDB.srv
    - mocha_core/srv/GetDataHeaderDB.srv
    - mocha_core/srv/SelectDB.srv
    """
    def __init__(self, robot_configs, topic_configs):
        """
        Initialize the DatabaseServer with robot and topic configurations.

        **Parameters:**
        - robot_configs (dict): Configuration dictionary containing all robots' settings
          Format: {
            'robot_name': {
              'node-type': 'ground_robot' | 'aerial_robot',
              'IP-address': '127.0.0.1',
              'base-port': '5005',
              'using-radio': True/False,
              'clients': ['other_robot_1', 'other_robot_2']
            }
          }

        - topic_configs (dict): Configuration dictionary defining which topics each
          robot type publishes and subscribes to
          Format: {
            'ground_robot': [
              {'msg_topic': '/robot/odometry', 'msg_type': 'nav_msgs/Odometry', ...},
              {'msg_topic': '/robot/scan', 'msg_type': 'sensor_msgs/LaserScan', ...}
            ],
            'aerial_robot': [...]
          }

        **Process:**
        1. Validates input configurations are dictionaries
        2. Gets this robot's name from ROS parameters (~robot_name)
        3. Validates robot_name exists in robot_configs
        4. Looks up robot's node-type and gets its topic list
        5. Creates a thread-safe database instance (DBwLock)
        6. Assigns a unique robot ID based on robot name
        7. Creates three ROS services for database operations
        8. Pre-processes message types for fast lookup during service calls
        """

        self.robot_configs = robot_configs
        self.topic_configs = topic_configs

        # Get current robot name from ROS parameter server (passed from launch file)
        self.robot_name = rospy.get_param('~robot_name')

        # Validate that this robot exists in the configuration
        if self.robot_name not in self.robot_configs.keys():
            rospy.logerr(f"{self.robot_name} does not exist in robot_configs")
            rospy.signal_shutdown("robot_name does not exist in robot_configs")
            rospy.spin()

        # Get the topic list for this robot's type (ground_robot or aerial_robot)
        # This list defines which topics this robot can publish/subscribe to
        self.topic_list = self.topic_configs[self.robot_configs[self.robot_name]["node-type"]]

        # Create the thread-safe database instance
        # DBwLock provides mutex/lock protection for concurrent access
        self.dbl = database.DBwLock()

        # Get a unique numeric ID for this robot based on its name
        # This ID is used as the first field in database messages for fast robot identification
        self.robot_number = du.get_robot_id_from_name(self.robot_configs,
                                                      robot_name=self.robot_name)

        # Create three ROS services that handle database operations
        # These services can be called by other nodes to interact with the database
        self.service_list = []
        
        # Service 1: AddUpdateDB - Add a new message or update existing one in database
        s = rospy.Service('~AddUpdateDB',
                          mocha_core.srv.AddUpdateDB,
                          self.add_update_db_service_cb)
        self.service_list.append(s)
        
        # Service 2: GetDataHeaderDB - Retrieve message content using its header/hash
        s = rospy.Service('~GetDataHeaderDB',
                          mocha_core.srv.GetDataHeaderDB,
                          self.get_data_hash_db_service_cb)
        self.service_list.append(s)
        
        # Service 3: SelectDB - Query/select messages by robot_id and topic_id (with filtering)
        s = rospy.Service('~SelectDB',
                          mocha_core.srv.SelectDB,
                          self.select_db_service_cb)
        self.service_list.append(s)

        # Pre-compute message types for all robots/topics for fast lookup during service calls
        # Format: {robot_id: {topic_id: {'dtype': data_type, ...}}}
        self.msg_types = du.msg_types(self.robot_configs, self.topic_configs)

    def add_update_db_service_cb(self, req):
        """
        Service callback for AddUpdateDB service.
        
        Adds a new message to the database or updates an existing one.
        The message is uniquely identified by (robot_id, topic_id, timestamp).
        
        **Request (AddUpdateDB.srv):**
        - topic_id (int): Index of the topic in the robot's topic_list [0, 1, 2, ...]
        - msg_content (bytes): Serialized message data (the actual sensor data/state)
        - timestamp (rospy.Time): Timestamp when the message was created/captured
        
        **Validation checks:**
        1. topic_id must be a valid integer (not None/empty)
        2. msg_content must not be empty (must contain actual data)
        3. topic_id must be within the robot's topic_list range (0 to len(topic_list)-1)
        
        **Database operation:**
        - Gets the message priority level from topic configuration
        - Converts rospy.Time to proper timestamp format
        - Creates a DBMessage object with metadata (robot_id, topic_id, data type, priority, timestamp, data)
        - Adds/updates the message in the thread-safe database
        - Database returns the message header (unique identifier for this message)
        
        **Response (AddUpdateDBResponse):**
        - header (bytes): Unique identifier for the stored message (used to retrieve it later)
        
        **Pattern:** Used by topic_publisher to store new sensor readings in the database
        """
        if not isinstance(req.topic_id, int) or req.topic_id is None:
            rospy.logerr("Error: topic_id empty")
            return
        if len(req.msg_content) == 0:
            rospy.logerr("Error: msg_content empty")
            return
        if req.topic_id > len(self.topic_list):
            rospy.logerr("Error: topic_id not in topic_list")
            return
        
        # Get topic configuration for this topic_id
        topic = self.topic_list[req.topic_id]
        
        # Convert message priority string (e.g., "HIGH_PRIORITY") to numeric value
        priority = du.get_priority_number(topic["msg_priority"])
        
        # Convert rospy.Time (with seconds and nanoseconds) to timestamp
        ts = req.timestamp
        ts = rospy.Time(ts.secs, ts.nsecs)
        
        # Create a database message object with all metadata
        dbm = database.DBMessage(self.robot_number,           # Which robot sent this
                                 req.topic_id,                 # Which topic/sensor
                                 dtype=self.msg_types[self.robot_number][req.topic_id]["dtype"],
                                 priority=priority,            # Priority level
                                 ts=ts,                        # When it was captured
                                 data=req.msg_content)         # The actual data

        # Add or update message in database (returns unique header/identifier)
        header = self.dbl.add_modify_data(dbm)
        
        # Return the header so caller can retrieve this message later
        return mocha_core.srv.AddUpdateDBResponse(header)

    def get_data_hash_db_service_cb(self, req):
        """
        Service callback for GetDataHeaderDB service.
        
        Retrieves the full message content (data + metadata) from the database
        using a message header/hash as the lookup key.
        
        A header uniquely identifies a message and is returned when you add/update
        a message via the AddUpdateDB service.
        
        **Request (GetDataHeaderDBRequest):**
        - msg_header (bytes): The unique identifier/header of the message to retrieve
          (obtained from AddUpdateDB response or previous GetDataHeaderDB call)
        
        **Validation:**
        - msg_header must not be None or empty
        
        **Database operation:**
        - Queries the thread-safe database to find the message with matching header
        - Returns: the message's metadata (robot_id, topic_id, timestamp) and data
        
        **Response (GetDataHeaderDBResponse):**
        - robot_id (int): Which robot sent this message
        - topic_id (int): Which topic/sensor this message came from
        - timestamp (rospy.Time): When the message was captured
        - data (bytes): The actual serialized message content
        
        **Pattern:** Used by translator to retrieve a specific message from the database
                    after receiving a message header from a remote robot
        """
        if req.msg_header is None or len(req.msg_header) == 0:
            rospy.logerr("Error: msg_header empty")
            return
        
        # Find the database message using its header/hash
        dbm = self.dbl.find_header(req.msg_header)
        
        # Create response with all message metadata and data
        answ = mocha_core.srv.GetDataHeaderDBResponse(dbm.robot_id,      # Source robot
                                                                dbm.topic_id,     # Source topic
                                                                dbm.ts,            # Timestamp
                                                                dbm.data)          # Actual data
        return answ

    def select_db_service_cb(self, req):
        """
        Service callback for SelectDB service.
        
        Queries/selects all messages from a specific robot in the database.
        Returns a list of message headers (identifiers) so caller can retrieve
        individual message contents.
        
        This is typical SQL-like "SELECT" operation:
        SELECT headers FROM database WHERE robot_id = X
        
        **Request (SelectDBRequest):**
        - robot_id (int): ID of the robot whose messages to select
        - topic_id (int): (Future) Filter by specific topic [NOT YET IMPLEMENTED]
        
        **Validation:**
        - robot_id must not be None (required)
        - topic_id must not be None (future implementation)
        
        **Database operation:**
        - Queries database to get all messages from the specified robot
        - Returns the header/identifier for each message (not the actual data)
        - Caller can use these headers with GetDataHeaderDB to fetch actual data
        
        **Response (SelectDBResponse):**
        - headers (bytes): Serialized list of all message headers from this robot
        
        **Future Enhancement (TODO):**
        - Implement topic_id filtering to select messages from specific topics only
        - Current behavior: returns all messages from a robot regardless of topic
        
        **Pattern:** Used by translator to get list of available messages from local or remote robots
                    Can then fetch each one individually as needed
        """
        # TODO: Implement filtering by topic_id for more selective queries
        
        if req.robot_id is None:
            rospy.logerr("Error: robot_id none")
            return
        if req.topic_id is None:
            rospy.logerr("Error: topic_id none")
            return
        
        # Get list of all message headers for this robot
        list_headers = self.dbl.get_header_list(req.robot_id)
        
        # Serialize the header list into a single bytes object
        answ = mocha_core.srv.SelectDBResponse(du.serialize_headers(list_headers))
        return answ

    def shutdown(self):
        """
        Cleanup method to gracefully shutdown the database server.
        
        Shuts down all ROS services that were created in __init__.
        This ensures clean shutdown when the node exits (e.g., catching Ctrl-C).
        
        **Actions:**
        - Iterates through all registered services (AddUpdateDB, GetDataHeaderDB, SelectDB)
        - Calls shutdown() on each service to stop accepting new requests
        - Allows in-flight requests to complete before full shutdown
        
        **Pattern:** Called from main script when receiving ROS shutdown signal
        """
        for s in self.service_list:
            s.shutdown()
