"""
DATABASE UTILITIES MODULE
=========================
This utility module provides helper functions for the MOCHA database system.

Key responsibilities:
1. Robot ID ↔ Robot Name mapping (bidirectional lookup)
2. Topic ID ↔ Topic Name mapping (bidirectional lookup)
3. Message serialization/deserialization for network transmission
4. Message type validation and extraction from configurations
5. Priority level conversion (string to numeric values)
6. Header pack/unpack operations for database messages

The database uses numeric IDs (robot_id, topic_id) for efficient storage and
network transmission, while configuration uses human-readable names. These
utilities provide bidirectional conversion between the two representations.

Dependencies:
- struct: Binary packing/unpacking for efficient network transmission
- rospy: ROS Python interface
- database (db): Custom database module with DBMessage class
- hash_comm: Header and timestamp operations
- lz4.frame: LZ4 compression for message payload compression
- importlib: Dynamic loading of ROS message types
"""

import struct
import rospy
import database as db
import hash_comm
import io
import pdb
import importlib
import random
import lz4.frame

# Constant defining the size of message headers (robot_id + topic_id + timestamp)
HEADER_LENGTH = hash_comm.TsHeader.HEADER_LENGTH


def get_robot_id_from_name(robot_configs, robot_name):
    """
    Maps a robot name (string) to its unique numeric ID.
    
    The robot_id is determined by the alphabetical position of the robot in
    the robot_configs dictionary (sorted by name). This ensures deterministic
    and consistent ID assignment across all runs.
    
    Example:
    - robot_configs = {'jackal_sim': {...}, 'uav': {...}}
    - sorted order: ['jackal_sim', 'uav']
    - get_robot_id_from_name(..., 'jackal_sim') returns 0
    - get_robot_id_from_name(..., 'uav') returns 1
    
    **Parameters:**
    - robot_configs (dict): Dictionary of all robots with their configurations
    - robot_name (str): Name of the robot to look up (e.g., 'jackal_sim', 'uav')
    
    **Returns:**
    - int: Numeric robot ID (0-based index in sorted robot list)
    
    **Raises:**
    - AssertionError: If robot_name is not in robot_configs
    
    **Uses:**
    - Database message creation (DBMessage requires numeric robot_id)
    - ROS service handlers that need to identify which robot sent a message
    - Network communication (headers use numeric IDs instead of strings for efficiency)
    """
    # Input validation
    assert isinstance(robot_configs, dict), "robot_configs must be a dictionary"
    assert robot_name is not None and isinstance(robot_name, str), "robot_name must be a non-empty string"

    # Get sorted list of robot names for deterministic ordering
    # Sorting ensures same ID assignment regardless of dict insertion order
    robot_list = list(robot_configs.keys())
    robot_list.sort()
    
    # robot_id is simply the index in the sorted list
    robot_id = robot_list.index(robot_name)
    return robot_id


def get_robot_name_from_id(robot_configs, robot_id):
    """
    Reverse mapping: converts a numeric robot ID back to its name (string).
    
    This is the inverse operation of get_robot_id_from_name().
    Used when receiving messages with numeric IDs and needing to get the actual
    robot name for human-readable logging or topic name construction.
    
    Example:
    - robot_configs = {'jackal_sim': {...}, 'uav': {...}}
    - get_robot_name_from_id(..., 0) returns 'jackal_sim'
    - get_robot_name_from_id(..., 1) returns 'uav'
    
    **Parameters:**
    - robot_configs (dict): Dictionary of all robots with their configurations
    - robot_id (int): Numeric ID of the robot to look up
    
    **Returns:**
    - str: Name of the robot (e.g., 'jackal_sim', 'uav')
    
    **Raises:**
    - AssertionError: If robot_id is not valid
    - KeyError: If robot_id not found in mapping
    
    **Uses:**
    - Constructing full topic names from headers (e.g., /jackal_sim/odometry/filtered)
    - Human-readable logging when displaying which robot sent a message
    - Translating network messages back to local topic structure
    """
    assert isinstance(robot_configs, dict), "robot_configs must be a dictionary"
    assert robot_id is not None and isinstance(robot_id, int), "robot_id must be an integer"
    
    # Create reverse mapping: for each robot, map its ID to its name
    # Uses dictionary comprehension with get_robot_id_from_name
    robots = {get_robot_id_from_name(robot_configs, robot): robot 
              for robot in robot_configs}
    
    # Look up the robot name using the numeric ID
    return robots[robot_id]


def get_topic_id_from_name(robot_configs, topic_configs,
                           robot_name, topic_name):
    """
    Maps a ROS topic name (string) to its numeric topic ID for a specific robot.
    
    Each robot has a list of topics it publishes/subscribes to. This function
    finds the index (topic_id) of a specific topic within that robot's topic list.
    
    Example (for jackal_sim which is a ground_robot):
    - Topics: [/jackal_sim/odometry/filtered, /jackal_sim/front/scan]
    - get_topic_id_from_name(..., 'jackal_sim', '/jackal_sim/odometry/filtered') returns 0
    - get_topic_id_from_name(..., 'jackal_sim', '/jackal_sim/front/scan') returns 1
    
    **Parameters:**
    - robot_configs (dict): Configuration of all robots
    - topic_configs (dict): Configuration mapping robot types to their topics
    - robot_name (str): Name of the robot (e.g., 'jackal_sim')
    - topic_name (str): Full ROS topic name (e.g., '/jackal_sim/odometry/filtered')
    
    **Returns:**
    - int: Index (0-based) of the topic in this robot's topic list
    
    **Raises:**
    - ROS signal shutdown: If topic_name not found in robot's topic list
    
    **Uses:**
    - Creating database messages with proper topic_id during serialization
    - Looking up message type information for a specific topic
    - Translating topic names to numeric IDs for network transmission
    """
    assert isinstance(robot_configs, dict)
    assert isinstance(topic_configs, dict)
    assert robot_name is not None and isinstance(robot_name, str)
    assert topic_name is not None and isinstance(topic_name, str)

    # Get the list of topics for this robot's type (ground_robot or aerial_robot)
    list_topics = topic_configs[robot_configs[robot_name]["node-type"]]
    
    # Search for the topic in the list
    id = None
    for i, topic in enumerate(list_topics):
        if topic_name == topic["msg_topic"]:
            id = i
            break
    
    # Handle case where topic is not found
    if id is None:
        rospy.logerr(f"{topic_name} does not exist in topic_configs")
        rospy.signal_shutdown("topic_name does not exist in topic_configs")
        rospy.spin()
    
    return id


def get_topic_name_from_id(robot_configs, topic_configs, robot_name, topic_id):
    """
    Reverse mapping: converts a numeric topic ID back to its ROS topic name.
    
    This is the inverse of get_topic_id_from_name().
    When receiving a message with numeric topic_id, this retrieves the actual
    topic name for publishing or subscribing.
    
    Example (for jackal_sim):
    - get_topic_name_from_id(..., 'jackal_sim', 0) returns '/jackal_sim/odometry/filtered'
    - get_topic_name_from_id(..., 'jackal_sim', 1) returns '/jackal_sim/front/scan'
    
    **Parameters:**
    - robot_configs (dict): Configuration of all robots
    - topic_configs (dict): Configuration mapping robot types to their topics
    - robot_name (str): Name of the robot (e.g., 'jackal_sim')
    - topic_id (int): Numeric index of the topic in this robot's topic list
    
    **Returns:**
    - str: Full ROS topic name (e.g., '/jackal_sim/odometry/filtered')
    
    **Raises:**
    - ROS signal shutdown: If topic_id is out of range
    
    **Uses:**
    - Reconstructing topic names when deserializing database messages
    - Publishing/subscribing to topics based on message headers
    - Network protocol: converting numeric IDs back to human-readable topics
    """
    assert isinstance(robot_configs, dict)
    assert isinstance(topic_configs, dict)
    assert robot_name is not None and isinstance(robot_name, str)
    assert topic_id is not None and isinstance(topic_id, int)
    
    # Get the list of topics for this robot's type
    list_topics = topic_configs[robot_configs[robot_name]["node-type"]]
    
    # Validate topic_id is within range
    if topic_id >= len(list_topics):
        rospy.logerr(f"{topic_id} does not exist in topic_configs")
        rospy.signal_shutdown("topic_id does not exist in topic_configs")
        rospy.spin()
    
    # Return the topic name at this index
    return list_topics[topic_id]["msg_topic"]


def serialize_headers(header_list):
    """
    Packs multiple message headers into a single bytes stream.
    
    Each header has a fixed size (HEADER_LENGTH bytes). This function concatenates
    multiple headers into one continuous byte stream for network transmission or storage.
    
    Example:
    - header_list = [b'header1' (32 bytes), b'header2' (32 bytes)]
    - Result: b'header1header2' (64 bytes total)
    - Can be reversed with deserialize_headers()
    
    **Parameters:**
    - header_list (list): List of header bytes, each of size HEADER_LENGTH
    
    **Returns:**
    - bytes: Concatenated stream of all headers
    
    **Uses:**
    - SelectDB service: packing all message headers to send to client
    - Network transmission: preparing header list for ZMQ messages
    
    **See also:**
    - deserialize_headers: Reverse operation
    """
    # Simple concatenation of all headers using bytes join
    return b''.join(header_list)


def deserialize_headers(serial_headers):
    """
    Unpacks a bytes stream back into individual message headers.
    
    Reverse of serialize_headers(). Splits a concatenated byte stream back into
    individual fixed-size headers.
    
    Example:
    - serial_headers = b'header1header2' (64 bytes)
    - Result: [b'header1' (32 bytes), b'header2' (32 bytes)]
    
    **Parameters:**
    - serial_headers (bytes): Concatenated stream of headers (must be multiple of HEADER_LENGTH)
    
    **Returns:**
    - list: List of individual header bytes (each of size HEADER_LENGTH)
    
    **Raises:**
    - Exception: If serial_headers length is not a multiple of HEADER_LENGTH
    
    **Uses:**
    - SelectDB client: unpacking header list received from database
    - Network reception: parsing incoming header lists from remote robots
    
    **See also:**
    - serialize_headers: Forward operation
    """
    # Validate that the stream length is correct (must be multiple of header size)
    if len(serial_headers) % HEADER_LENGTH != 0:
        raise Exception('deserialize_hashes: wrong length of string')
    
    # Split stream into chunks of HEADER_LENGTH bytes each
    splitted_headers = [serial_headers[i:i+HEADER_LENGTH]
                        for i in
                        range(0, len(serial_headers), HEADER_LENGTH)]
    return splitted_headers

def pack_data(msg):
    """
    Packs a database message into a standard binary transmission format.
    
    Converts a DBMessage object into a compact binary representation suitable for
    network transmission. Only the data fields are packed; header fields (robot_id,
    topic_id, timestamp) are handled separately by hash_comm module.
    
    **Binary Format (total size = 2 bytes + data size):**
    [
      byte 0: data_dtype (1 unsigned char)       - message type ID (0-255)
      byte 1: priority (1 unsigned char)         - priority level (0-3)
      bytes 2+: data (variable)                  - serialized message payload
    ]
    
    Example:
    - msg = DBMessage(robot_id=0, topic_id=1, dtype=5, priority=3, ts=..., data=b'payload')
    - Result: b'\x05\x03' + b'payload' (2 bytes header + payload)
    
    **Parameters:**
    - msg (database.DBMessage): The database message to pack
    
    **Returns:**
    - bytes: Binary packed data in the standard transmission format
    
    **Raises:**
    - AssertionError: If msg is not a DBMessage instance
    
    **Uses:**
    - Translator module: packing data before sending to remote robot via ZMQ
    - Network transmission: creating binary packets for efficient transmission
    
    **See also:**
    - unpack_data: Reverse operation
    """
    assert isinstance(msg, db.DBMessage), "msg must be a DBMessage instance"
    
    # Pack dtype as unsigned char (1 byte, network byte order)
    serial_data = struct.pack('!B', msg.dtype)
    
    # Pack priority as unsigned char (1 byte, network byte order)
    serial_data += struct.pack('!B', msg.priority)
    
    # Append raw message data (already serialized by ROS)
    serial_data += msg.data
    
    return serial_data


def unpack_data(header, packed_data):
    """
    Unpacks a binary data packet back into a DBMessage object.
    
    Reverse of pack_data(). Reconstructs a DBMessage from its packed binary representation
    by extracting header information separately and unpacking the data payload.
    
    **Binary Format (matches pack_data output):**
    [
      byte 0: data_dtype (1 unsigned char)       - message type ID
      byte 1: priority (1 unsigned char)         - priority level
      bytes 2+: data (variable)                  - serialized message payload
    ]
    
    Example:
    - header = b'<32-byte header with robot_id=0, topic_id=1, timestamp>'
    - packed_data = b'\x05\x03payload_bytes'
    - Result: DBMessage(robot_id=0, topic_id=1, dtype=5, priority=3, ts=..., data=b'payload_bytes')
    
    **Parameters:**
    - header (bytes): Fixed-size header containing (robot_id, topic_id, timestamp)
                      (typically 32 bytes, format defined in hash_comm.TsHeader)
    - packed_data (bytes): Binary data packet starting with dtype and priority bytes
    
    **Returns:**
    - database.DBMessage: Reconstructed database message object
    
    **Raises:**
    - AssertionError: If data section is empty
    
    **Process:**
    1. Extract header information (robot_id, topic_id, timestamp)
    2. Unpack dtype from byte 0
    3. Unpack priority from byte 1
    4. Extract remaining bytes as the message data
    5. Create and return DBMessage object
    
    **Uses:**
    - Translator module: unpacking received data from remote robot
    - Network reception: recreating DBMessages from binary packets
    - Database insertion: converting network messages back to database format
    
    **See also:**
    - pack_data: Forward operation
    """
    # Parse the header to get robot_id, topic_id, and timestamp
    h = hash_comm.TsHeader.from_header(header)
    p_robot_id, p_topic_id, p_ts = h.get_id_and_time()

    # Track position in the packed_data buffer
    pointer = 0
    
    # Unpack topic data type (1 byte, network byte order)
    p_topic_dtype = struct.unpack('!B',
                                    packed_data[pointer:pointer + 1])[0]
    pointer += 1
    
    # Unpack priority (1 byte, network byte order)
    p_priority = struct.unpack('!B',
                               packed_data[pointer:pointer + 1])[0]
    pointer += 1
    
    # Extract remaining bytes as the message data
    p_data = packed_data[pointer:]
    assert len(p_data) != 0, "Empty data in unpack_data"

    # Reconstruct the DBMessage object with all extracted information
    dbm = db.DBMessage(p_robot_id,      # Which robot sent this
                       p_topic_id,      # Which topic/sensor
                       p_topic_dtype,   # Message type ID
                       p_priority,      # Priority level
                       p_ts,            # Timestamp
                       p_data)          # Serialized message data
    return dbm


def serialize_ros_msg(msg):
    """
    Serializes a ROS message object into compressed binary format.
    
    Converts a ROS message (nav_msgs/Odometry, sensor_msgs/LaserScan, etc.)
    into its binary representation and compresses it using LZ4 for efficient
    network transmission.
    
    **Process:**
    1. Use ROS message's serialize() method to get binary representation
    2. Compress the binary data using LZ4 frame format
    3. Return compressed bytes
    
    **Compression details:**
    - LZ4 provides fast compression/decompression (good for real-time systems)
    - Typical compression ratio: 50-70% for sensor data depending on payload
    - Fast enough for high-frequency data streams
    
    Example:
    - Input: nav_msgs/Odometry message object
    - Serialized size: ~200 bytes
    - Compressed size: ~100 bytes (50% reduction)
    
    **Parameters:**
    - msg: Any ROS message object (nav_msgs/Odometry, sensor_msgs/LaserScan, etc.)
    
    **Returns:**
    - bytes: LZ4-compressed binary data
    
    **Uses:**
    - Topic publisher: compressing ROS messages before storing in database
    - Network transmission: reducing bandwidth for multi-robot communication
    - Database storage: reducing storage requirements
    
    **See also:**
    - parse_answer: Decompression operation
    
    **TODO:**
    - Add validation to ensure message is not garbage/corrupted
    """
    # Create bytes buffer for serialization
    sio_h = io.BytesIO()
    
    # Serialize ROS message to buffer (gives uncompressed binary)
    msg.serialize(sio_h)
    serialized = sio_h.getvalue()
    
    # Compress using LZ4 for efficient transmission
    compressed = lz4.frame.compress(serialized)
    return compressed


def parse_answer(api_answer, msg_types):
    """
    Parses an API response containing a ROS message and reconstructs the message object.
    
    Reverse of serialize_ros_msg(). Takes a response from the database API containing
    compressed message data and returns the deserialized ROS message object along with
    metadata (robot_id, topic_id, timestamp).
    
    **Process:**
    1. Look up message class constructor based on robot_id and topic_id
    2. Create an empty message object instance
    3. Decompress the message content using LZ4
    4. Deserialize binary data into the message object
    5. Return message object and metadata
    
    Example:
    - api_answer contains: compressed_odometry_data with robot_id=0, topic_id=0
    - msg_types[0][0] = {'obj': Odometry class, ...}
    - Result: (robot_id=0, topic_id=0, timestamp=..., msg=Odometry_object)
    
    **Parameters:**
    - api_answer: Response from GetDataHeaderDB service containing:
      - robot_id (int): Source robot
      - topic_id (int): Source topic
      - msg_content (bytes): LZ4-compressed serialized message
      - timestamp (rospy.Time): When message was captured
    - msg_types (dict): Lookup table mapping (robot_id, topic_id) to message class
      Format: msg_types[robot_id][topic_id] = {'obj': MessageClass, 'dtype': int, 'name': str}
    
    **Returns:**
    - tuple: (robot_id, topic_id, timestamp, msg)
      - robot_id (int): Which robot sent this
      - topic_id (int): Which topic it came from
      - timestamp (rospy.Time): When it was captured
      - msg: Deserialized ROS message object (Odometry, LaserScan, etc.)
    
    **Uses:**
    - Topic publisher: decompressing and reconstructing ROS messages from database
    - Translator module: recovering messages received from remote robots
    - Application: accessing actual sensor data in ROS message format
    
    **See also:**
    - serialize_ros_msg: Forward operation
    """
    # Get the message class constructor for this robot/topic combination
    constructor = msg_types[api_answer.robot_id][api_answer.topic_id]['obj']
    
    # Create an empty message object of the correct type
    msg = constructor()
    
    # Decompress: api_answer.msg_content is LZ4-compressed binary
    decompressed = lz4.frame.decompress(api_answer.msg_content)
    
    # Deserialize: convert binary back to message object
    msg.deserialize(decompressed)
    
    # Extract and return all relevant information
    robot_id = api_answer.robot_id
    topic_id = api_answer.topic_id
    ts = api_answer.timestamp
    
    return robot_id, topic_id, ts, msg


def msg_types(robot_configs, topic_configs):
    """
    Extracts and validates message types from configurations, creates a lookup table.
    
    This function builds a comprehensive index of all ROS message types used by all robots.
    It validates message type names, dynamically loads message classes, and creates a
    mapping structure for efficient lookup during message processing.
    
    **Validation Checks:**
    1. Message type format: Must be "package_name/MessageClass" (exactly 2 parts)
    2. Character validation: Only alphanumeric and underscore characters allowed
    
    **Process:**
    1. Collect all unique message types from all robot topic configurations
    2. Validate each message type format
    3. Sort message types alphabetically for deterministic ordering
    4. Assign each unique message type a numeric ID (0, 1, 2, ...)
    5. Dynamically load each message class using importlib
    6. Build lookup structure: msg_types[robot_id][topic_id] = {...}
    
    **Output Structure:**
    msg_types = {
      robot_id: {
        topic_id: {
          'dtype': int,                  # Unique ID for this message type (0-255)
          'obj': MessageClass,           # The actual Python class (e.g., Odometry)
          'name': 'package_name/MessageClass'
        }
      }
    }
    
    **Example:**
    Given config:
    - robot_configs = {'jackal_sim': {node-type: 'ground_robot'}, 'uav': {node-type: 'aerial_robot'}}
    - topic_configs = {
        'ground_robot': [
          {'msg_type': 'nav_msgs/Odometry', ...},
          {'msg_type': 'sensor_msgs/LaserScan', ...}
        ],
        'aerial_robot': [
          {'msg_type': 'nav_msgs/Odometry', ...},  # Same type as ground_robot
          {'msg_type': 'sensor_msgs/Image', ...}
        ]
      }
    
    Result:
    - All unique types sorted: ['nav_msgs/Image', 'nav_msgs/Odometry', 'sensor_msgs/LaserScan']
    - msg_types[0][0] = {'dtype': 1, 'obj': Odometry, 'name': 'nav_msgs/Odometry'}
    - msg_types[0][1] = {'dtype': 2, 'obj': LaserScan, 'name': 'sensor_msgs/LaserScan'}
    - msg_types[1][0] = {'dtype': 1, 'obj': Odometry, 'name': 'nav_msgs/Odometry'}  (Same dtype!)
    - msg_types[1][1] = {'dtype': 0, 'obj': Image, 'name': 'sensor_msgs/Image'}
    
    **Parameters:**
    - robot_configs (dict): Configuration of all robots with their node-types
    - topic_configs (dict): Mapping of node-type to list of topics and their message types
    
    **Returns:**
    - dict: Nested dictionary mapping robot_id → topic_id → message type information
    
    **Raises:**
    - ROS signal shutdown: If message type format is invalid
    - ImportError: If message class cannot be imported (ROS package issue)
    - AttributeError: If message class doesn't exist in package
    
    **Important Notes:**
    - Message types are sorted to ensure deterministic dtype assignment across all runs
    - The same message type used by multiple robots gets the same dtype value
    - This allows efficient network transmission (dtype is 1 byte, 0-255 possible)
    - Must be called once at startup, result cached for entire runtime
    
    **Uses:**
    - Database server: mapping message types during pack_data/unpack_data
    - Topic publisher: looking up message class for deserialization
    - Configuration validation: checking that message types exist and are valid
    
    **Example Usage:**
    msg_types_lookup = msg_types(robot_configs, topic_configs)
    # Later: construct message for robot 0, topic 1
    msg_class = msg_types_lookup[0][1]['obj']
    msg_instance = msg_class()
    """
    assert isinstance(topic_configs, dict), "topic_configs must be a dictionary"

    # Collect all unique message types across all robots and topics
    msg_list = []
    for robot in topic_configs:
        for topic in topic_configs[robot]:
            msg = topic['msg_type']
            
            # Validate message type format: must be "package/MessageName"
            parts = msg.split('/')
            if not (len(parts) == 2 and
                    all(part.replace("_", "").isalnum()
                        for part in parts)):
                rospy.logerr(f"Error: msg_type {msg} not valid")
                rospy.signal_shutdown("Error: msg_type {msg} not valid")
                rospy.spin()
            
            msg_list.append(topic['msg_type'])
    
    # Sort message types alphabetically for deterministic dtype assignment
    # This ensures consistent type IDs across all runs and robots
    msg_list.sort()

    # Build the lookup structure: msg_types[robot_id][topic_id] = {...}
    msg_types = {}
    
    for robot in robot_configs:
        # Convert robot name to numeric ID
        robot_id = get_robot_id_from_name(robot_configs, robot)
        
        # Initialize this robot's entry if not already present
        if robot_id not in msg_types.keys():
            msg_types[robot_id] = {}
        
        # Process each topic for this robot
        for i, topic in enumerate(topic_configs[robot_configs[robot]["node-type"]]):
            msg = topic['msg_type']
            
            # Find the dtype: index in the globally sorted message type list
            # This ensures same message type gets same dtype across robots
            msg_id = msg_list.index(msg)
            
            # Dynamically import the message class
            # Example: 'nav_msgs/Odometry' → import nav_msgs.msg, get Odometry class
            package_name, msg_name = msg.split('/')
            package = importlib.import_module(package_name + '.msg')
            message_type = getattr(package, msg_name)
            
            # Store message type information for fast lookup
            msg_types[robot_id][i] = {
                "dtype": msg_id,           # Global type ID (0-255)
                "obj": message_type,       # Actual Python message class
                "name": msg                # Human-readable name "package/Class"
            }
    
    return msg_types

def get_priority_number(priority_string):
    """
    Converts a priority string to its numeric value.
    
    MOCHA uses priority levels to determine which messages to transmit first
    over bandwidth-limited networks. This function maps human-readable priority
    strings to numeric values (0-3).
    
    **Priority Levels (0=lowest, 3=highest):**
    - NO_PRIORITY (0):       Not used in transmission, informational only
    - LOW_PRIORITY (1):      Background data, transmit when bandwidth available
    - MEDIUM_PRIORITY (2):   Regular data (odometry, status updates)
    - HIGH_PRIORITY (3):     Critical data (command acknowledgments, alarms)
    
    **Use Cases:**
    - High priority: Emergency stop, collision warning, critical pose updates
    - Medium priority: Normal odometry, IMU data, status messages
    - Low priority: Compressed images, raw sensor logs, debug data
    
    **Example:**
    - get_priority_number('HIGH_PRIORITY') returns 3
    - get_priority_number('MEDIUM_PRIORITY') returns 2
    - get_priority_number('LOW_PRIORITY') returns 1
    
    **Parameters:**
    - priority_string (str): One of {'NO_PRIORITY', 'LOW_PRIORITY', 'MEDIUM_PRIORITY', 'HIGH_PRIORITY'}
    
    **Returns:**
    - int: Numeric priority value (0-3)
    
    **Raises:**
    - AssertionError: If priority_string is not a valid string in PRIORITY_LUT
    
    **Uses:**
    - Topic configuration: extracting priority from topic_configs.yaml
    - Database message creation: assigning priority to messages
    - Translator/synchronizer: selecting which messages to transmit based on priority
    - Bandwidth throttling: dropping low-priority messages first when bandwidth is limited
    """
    # Priority lookup table mapping string names to numeric values
    PRIORITY_LUT = {"NO_PRIORITY": 0,
                    "LOW_PRIORITY": 1,
                    "MEDIUM_PRIORITY": 2,
                    "HIGH_PRIORITY": 3}
    
    assert isinstance(priority_string, str), "priority_string must be a string"
    assert priority_string in PRIORITY_LUT, f"priority_string '{priority_string}' not in {list(PRIORITY_LUT.keys())}"
    
    return PRIORITY_LUT[priority_string]


def generate_random_header():
    """
    Generates a random message header for testing purposes.
    
    Creates a random but valid header with random robot_id, topic_id, and timestamp.
    Used primarily for unit testing, debugging, or simulating messages without
    requiring actual sensor data.
    
    **Generated Random Values:**
    - robot_id: Random int from 0-255 (255 possible robots)
    - topic_id: Random int from 0-255 (255 possible topics per robot)
    - timestamp: Random rospy.Time (0 to ~1 second)
    
    **Example:**
    - First call: header = b'\x42\x7C\x00\x00...' (random bytes)
    - Second call: header = b'\x15\x9A\x00\x00...' (different random bytes)
    
    **Returns:**
    - bytes: Binary header (typically 32 bytes, format defined by hash_comm.TsHeader)
    
    **Uses:**
    - Unit testing: creating test messages without accessing real database
    - Debugging: simulating messages from multiple robots
    - Prototyping: testing MOCHA framework without hardware
    
    **Note:**
    - Should not be used in production code where actual message data is needed
    - Useful for testing database operations, serialization, network transmission
    """
    # Generate random robot_id (0-255)
    robot_id = random.randint(0, 255)
    
    # Generate random topic_id (0-255)
    topic_id = random.randint(0, 255)
    
    # Generate random timestamp (0 to ~1 second in ROS time)
    time = rospy.Time.from_sec(random.random())
    
    # Create header from the random values
    h = hash_comm.TsHeader.from_data(robot_id, topic_id, time)
    
    # Return the binary representation (digest) of the header
    return h.bindigest()
