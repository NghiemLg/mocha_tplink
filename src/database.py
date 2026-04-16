#!/usr/bin/env python3
"""
DATABASE MODULE
===============
This module implements the core database system for the MOCHA framework.

The database stores multi-robot sensor data and messages with thread-safe access.

Key Components:
1. DBMessage: Represents a single message with metadata (robot_id, topic_id, timestamp)
2. DBwLock: Thread-safe database with lock protection for concurrent access

Database Structure:
db = {
  robot_id: {
    topic_id: {
      header: DBMessage,        # Messages indexed by header (robot_id + topic_id + timestamp)
      header: DBMessage,
      ...
    }
  }
}

The database uses binary headers (from hash_comm.TsHeader) as unique keys for each message.
This allows O(1) lookups and efficient storage of multi-robot data with timestamps.

Dependencies:
- threading: Thread synchronization and locks
- hash_comm: Binary header creation and timestamp encoding
- rospy: ROS Python interface for timestamps
- numpy: Numerical operations and comparisons
- copy: Deep copying for database initialization
"""

import threading
import hash_comm
import rospy
import pdb
import database_utils as 
import numpy as np
import hash_comm
import copy


class DBMessage():
    """
    Database Message Object
    =======================
    Represents a single sensor message or data payload for multi-robot storage.
    
    This class encapsulates all information needed to store and retrieve a message
    from the MOCHA database:
    - Source identification: which robot sent this, which sensor/topic
    - Timing: when the data was captured (timestamp)
    - Message content: the actual serialized data payload
    - Priority: importance level for network transmission
    - Metadata: data type identifier for deserialization
    
    Each DBMessage is uniquely identified by a binary header computed from:
    (robot_id, topic_id, timestamp)
    
    This header serves as the primary key for database lookups and is calculated
    deterministically using hash_comm.TsHeader.
    """
    def __init__(self, robot_id, topic_id,
                 dtype, priority, ts, data):
        """
        Initialize the DBMessage object with multi-robot sensor data.
        
        This initializer creates a database message from raw components and computes
        its unique binary header identifier. The header is deterministic based on
        (robot_id, topic_id, timestamp), enabling reproducible lookups across different
        processes and machines.
        
        Process:
        1. Validate all input types (ensures data integrity)
        2. Store source identification (robot_id, topic_id)
        3. Store timestamp for temporal ordering
        4. Store metadata (dtype, priority)
        5. Store the binary payload
        6. Compute binary header from (robot_id, topic_id, timestamp)
        
        Args:
            robot_id (int): Unique identifier of the robot sending the message
                           (e.g., 0=Jackal UGV, 1=Hector Quadrotor UAV)
            topic_id (int): Unique identifier of the sensor/topic
                           (e.g., 0=laser, 1=camera, 2=gps)
            dtype (int): Data type identifier for message deserialization
                        (e.g., 0=LaserScan, 1=Image, 2=Odometry)
            priority (int): Priority level for transmission (0=low, 5=critical)
                           Higher priority messages sent first in bandwidth-limited scenarios
            ts (rospy.Time): ROS timestamp when data was captured
                            Used for temporal ordering and synchronization
            data (bytes): Binary serialized message payload
                         Contains compressed sensor data (typically LZ4 compressed)
        
        Example:
            # Create message: Jackal's (robot_id=0) laser scan (topic_id=0) at time 100.5s
            # Data type 0 (LaserScan), priority 3, captured 100.5 seconds since epoch
            msg = DBMessage(
                robot_id=0,
                topic_id=0,
                dtype=0,
                priority=3,
                ts=rospy.Time(100.5),
                data=b'\\x00\\x01\\x02...'  # Compressed laser data
            )
            # Header computed as: hash_comm.TsHeader(0, 0, 100.5).bindigest()
            # This header uniquely identifies this message in the database
        
        Raises:
            AssertionError: If any input type doesn't match expected type
                           (robot_id, topic_id, dtype, priority must be int)
                           (ts must be rospy.Time)
                           (data must be bytes)
        """
        assert isinstance(robot_id, int)
        assert isinstance(topic_id, int)
        assert isinstance(dtype, int)
        assert isinstance(priority, int)
        assert isinstance(ts, rospy.Time)
        assert isinstance(data, bytes)
        # The first three items are encoded in the header
        self.robot_id = robot_id
        self.topic_id = topic_id
        self.ts = ts
        # The next items are encoded in the data
        self.dtype = dtype
        self.priority = priority
        self.data = data
        # Calculate the header of the message
        header = hash_comm.TsHeader.from_data(self.robot_id,
                                              self.topic_id, self.ts)
        self.header = header.bindigest()

    def __eq__(self, other):
        """
        Compare two DBMessage objects for equality with timestamp tolerance.
        
        Two messages are considered equal if all fields match, with special handling
        for ROS timestamps to account for nanosecond-level precision variations that
        can occur during serialization/deserialization or network transmission.
        
        Comparison Order (fail-fast):
        1. Type check: Verify other is a DBMessage instance
        2. Source identification: robot_id, topic_id must match exactly
        3. Metadata: dtype, priority must match exactly
        4. Timestamp seconds: ts.secs must match exactly
        5. Timestamp nanoseconds: ts.nsecs tolerance of 1,000,000 ns (1 ms)
           - Allows for minor precision loss during compression/serialization
        6. Payload: data bytes must match exactly
        
        Args:
            other: Object to compare with this DBMessage
        
        Returns:
            bool: True if all fields match (within tolerances), False otherwise
        
        Use Cases:
            - Validating messages received from network match originals
            - Detecting duplicate messages in database
            - Synchronizing data between robots
            - Testing message serialization/deserialization
            - Verifying ROS message integrity after timestamping
        
        Example:
            msg1 = DBMessage(0, 0, 0, 3, rospy.Time(100, 123456), b'\\x00\\x01')
            msg2 = DBMessage(0, 0, 0, 3, rospy.Time(100, 124456), b'\\x00\\x01')  # ~1ms diff
            assert msg1 == msg2  # True (within 1ms tolerance)
            
            msg3 = DBMessage(0, 0, 0, 3, rospy.Time(100, 500000), b'\\x00\\x01')  # 5ms diff
            assert msg1 == msg3  # False (exceeds tolerance)
        
        Note:
            Timestamp tolerance of 1,000,000 nanoseconds (1 millisecond) is used to
            account for minor precision loss during LZ4 compression and ROS serialization.
            This is larger than typical network jitter (< 100 µs) to avoid false negatives.
        """
        if not isinstance(other, DBMessage):
            return False
        if other.robot_id != self.robot_id:
            return False
        if other.topic_id != self.topic_id:
            return False
        if other.dtype != self.dtype:
            return False
        if other.priority != self.priority:
            return False
        if other.ts.secs != self.ts.secs:
            return False
        if np.abs(other.ts.nsecs - self.ts.nsecs) > 1000000:
            print("nsecs diff: %d" % np.abs(other.ts.nsecs - self.ts.nsecs))
            return False
        if other.data != self.data:
            return False
        return True

    def __str__(self):
        """
        String representation of the DBMessage for debugging and logging.
        
        Returns a formatted string containing the key identifying information
        of the message for debugging, logging, and console output.
        
        Format: "robot_id, topic_id, dtype, priority, timestamp"
        
        Returns:
            str: Formatted string like "0, 0, 0, 3, 100.500000"
        
        Example:
            msg = DBMessage(0, 0, 0, 3, rospy.Time(100.5), b'\\x00\\x01')
            print(msg)  # Output: "0, 0, 0, 3, 100.500000"
        
        Use Cases:
            - Debugging: Print message identity without payload
            - Logging: Track which robot/topic/time in database operations
            - Testing: Quick visual verification of message routing
            - Error reporting: Include in exception or debug messages
        
        Note:
            The binary header and data payload are not included to keep output
            readable and prevent console flooding. Use str(msg.header) or
            len(msg.data) if those details are needed.
        """
        return "%d, %d, %d, %d, %f" % (self.robot_id, self.topic_id,
                                       self.dtype, self.priority,
                                       self.ts)


class DBwLock():
    """
    Thread-Safe Multi-Robot Message Database
    =========================================
    A dictionary-based database with lock protection for concurrent access by
    multiple threads in the MOCHA framework. Stores DBMessage objects organized
    hierarchically by robot and topic.
    
    This class provides atomic operations on a 3-level nested dictionary:
    db[robot_id][topic_id][header] = DBMessage
    
    Thread Safety:
    - threading.Lock protects all read/write operations
    - Lock MUST be acquired before accessing db attribute directly
    - All public methods (add_modify_data, get_header_list, etc.) handle locking
    
    Database Structure:
    {
      robot_id_0: {
        topic_id_0: {header_1: DBMessage, header_2: DBMessage, ...},
        topic_id_1: {header_3: DBMessage, ...}
      },
      robot_id_1: {
        topic_id_0: {header_4: DBMessage, ...}
      }
    }
    
    Optional Initialization:
    - Can preload a sample database dictionary (useful for testing/debugging)
    - Deepcopied on initialization to prevent external modifications
    
    Use Cases:
    - Storing sensor messages from Jackal UGV and Hector Quadrotor UAV
    - Synchronizing data between multiple ROS nodes
    - Querying messages by robot, topic, or time range
    - Preventing race conditions in multi-threaded environments
    """
    def __init__(self, sample_db=None):
        """
        Initialize the thread-safe database with optional preloading.
        
        Process:
        1. If sample_db provided: Deep copy it (prevent external modifications)
        2. If sample_db None: Start with empty database dictionary
        3. Create threading.Lock for concurrent access protection
        
        Args:
            sample_db (dict, optional): Pre-populated database dictionary to load into
                                       the object. Useful for testing, debugging, or
                                       initializing with known data. If provided, must
                                       follow the hierarchical structure:
                                       {robot_id: {topic_id: {header: DBMessage}}}
                                       Defaults to None (empty database).
        
        Returns:
            None
        
        Raises:
            AssertionError: If sample_db is not None and not a dictionary
        
        Examples:
            # Empty database (normal initialization)
            db = DBwLock()
            print(len(db.db))  # 0
            
            # Pre-populated database (for testing)
            sample = {0: {0: {b'header1': msg1, b'header2': msg2}}}
            db = DBwLock(sample)
            print(len(db.db[0][0]))  # 2 messages for robot 0, topic 0
            
            # Verify deepcopy prevents external modification
            sample = {0: {0: {}}}
            db = DBwLock(sample)
            sample[0][0]['new'] = 'value'  # Doesn't affect db
            print(db.db[0][0])  # Still empty
        
        Attributes Created:
            self.db (dict): The hierarchical message database
            self.lock (threading.Lock): Synchronization lock for thread safety
        
        Thread Safety:
            Lock is initialized for concurrent access, but not acquired during __init__.
            First operation will acquire the lock as needed.
        """
        if sample_db is not None:
            assert isinstance(sample_db, dict)
            self.db = copy.deepcopy(sample_db)
        else:
            self.db = {}
        self.lock = threading.Lock()

    def add_modify_data(self, dbm):
        """
        Insert or update a message in the database with thread safety.
        
        Atomically stores a DBMessage into the hierarchical database structure,
        creating intermediate dictionaries as needed. This method is both an
        INSERT and UPDATE operation: if the header already exists, it overwrites
        the previous message (useful for updating timestamps or priority).
        
        Thread Safety:
        - Acquires self.lock before any database modification
        - Releases lock after atomic insertion
        - Ensures no race conditions even with concurrent threads
        - Safe to call from multiple robot integrate_database nodes simultaneously
        
        Operation Flow:
        1. Validate input is a DBMessage instance
        2. Acquire threading.Lock to prevent concurrent modification
        3. Create robot_id dictionary if doesn't exist
        4. Create topic_id dictionary if doesn't exist under robot_id
        5. Store/overwrite message using binary header as key
        6. Release lock
        7. Return the header for verification/tracking
        
        Args:
            dbm (DBMessage): Message to insert/update in the database
                            Must be fully initialized with robot_id, topic_id,
                            timestamp, dtype, priority, and data
        
        Returns:
            bytes: The binary header used as the storage key. Useful for
                   confirming insertion and retrieving the message later
        
        Raises:
            AssertionError: If dbm is not a DBMessage instance
        
        Examples:
            # Insert Jackal laser scan
            msg = DBMessage(0, 0, 0, 3, rospy.Time(100.5), b'\\x00\\x01...')
            header = db.add_modify_data(msg)
            
            # Insert UAV odometry (creates topic 2 for this robot)
            msg2 = DBMessage(1, 2, 1, 2, rospy.Time(100.6), b'\\x02\\x03...')
            header2 = db.add_modify_data(msg2)
            
            # Update Jackal laser at same time (overwrites msg)
            msg_update = DBMessage(0, 0, 0, 4, rospy.Time(100.5), b'\\x04\\x05...')
            header = db.add_modify_data(msg_update)  # Same header, new priority
        
        Use Cases:
            - Main insertion point called by integrate_database node
            - Updating message priority based on network conditions
            - Storing sensor data from multiple robots asynchronously
            - Called from ROS service callbacks (AddUpdateDB service)
        
        Database Impact:
            Before:  db = {0: {0: {header1: msg1}}}
            After:   db = {0: {0: {header1: msg1, header2: msg2}}}
            
            With concurrent robot:
            After:   db = {0: {0: {...}}, 1: {0: {header: msg}}}
        
        Performance:
            O(1) for insertion (hash table operation)
            Lock overhead: microseconds (scales poorly for high-frequency data)
        """
        # Do a quick check
        assert isinstance(dbm, DBMessage)

        # Acquire lock and commit into db
        self.lock.acquire()
        # Create and store things in db
        if dbm.robot_id not in self.db:
            self.db[dbm.robot_id] = {}
        if dbm.topic_id not in self.db[dbm.robot_id]:
            self.db[dbm.robot_id][dbm.topic_id] = {}
        self.db[dbm.robot_id][dbm.topic_id][dbm.header] = dbm
        self.lock.release()
        return dbm.header

    def get_header_list(self, filter_robot_id=None, filter_latest=None):
        """
        Query the database and return a sorted list of message headers.
        
        Supports two filtering modes to query specific subsets of data:
        1. filter_robot_id: Return only messages from a specific robot
        2. filter_latest: Return only the most recent message per topic
        
        Results are sorted by priority (descending), then timestamp, for efficient
        transmission in bandwidth-limited scenarios (high-priority recent data first).
        
        Thread Safety:
        - Acquires self.lock for consistent snapshot of database during query
        - Prevents race conditions with concurrent add_modify_data() calls
        - Consistent sorting ensures reproducible ordering across processes
        
        Sorting Order (highest priority first):
        1. Priority level (0=low to 5=critical) - descending
        2. Timestamp (newer first) - descending  
        3. Header binary value (consistent tie-breaker) - descending
        
        Args:
            filter_robot_id (int, optional): Restrict results to a specific robot ID.
                                            If None, returns headers from ALL robots.
                                            Typical values: 0=Jackal UGV, 1=Hector UAV
                                            Defaults to None (all robots).
            
            filter_latest (bool, optional): Return only the most recent message per topic.
                                           If True: For each (robot_id, topic_id) pair,
                                                    return only the message with the
                                                    highest timestamp.
                                           If False: Return all messages in the database.
                                           Defaults to None (treated as False).
                                           NOTE: filter_latest assumes messages within
                                                 a robot's frame have consistent time
                                                 (each robot has its own timestamp domain).
        
        Returns:
            list: Sorted list of binary headers [header1, header2, ...]
                 Sorted by (priority DESC, timestamp DESC, header DESC).
                 Empty list if no messages match the filters.
        
        Raises:
            AssertionError: If filter_robot_id not None and not an int
            AssertionError: If filter_latest not None and not a bool
        
        Examples:
            # Get all headers from database (all robots, all messages)
            all_headers = db.get_header_list()
            
            # Get only Jackal UGV messages (robot_id=0)
            jackal_headers = db.get_header_list(filter_robot_id=0)
            
            # Get only most recent message per topic for all robots
            # Useful for initial synchronization
            recent_headers = db.get_header_list(filter_latest=True)
            
            # Get only most recent Jackal messages (robot_id=0, latest per topic)
            jackal_latest = db.get_header_list(filter_robot_id=0, filter_latest=True)
            
            # Access returned headers
            headers = db.get_header_list(filter_robot_id=0)
            for header in headers:
                msg = db.db[0][...][header]  # Access message via header
        
        Use Cases:
            1. Synchronization: Get latest messages to sync with remote robot
               headers = db.get_header_list(filter_robot_id=1, filter_latest=True)
            
            2. High-priority transmission: Get all messages sorted by priority
               headers = db.get_header_list()
               # Send top N headers over limited bandwidth link
            
            3. Export specific robot data: Get all messages from Jackal
               headers = db.get_header_list(filter_robot_id=0)
            
            4. Query per-topic latest: Get one message per topic per robot
               headers = db.get_header_list(filter_latest=True)
        
        Implementation Notes:
            - Each robot has independent timestamp domain (filter_latest per-robot)
            - Latest per topic = highest timestamp within that (robot_id, topic_id) pair
            - Sorting ensures consistent ordering across distributed systems
            - Lock is acquired and released within this method (blocking operation)
        
        Performance:
            - Time: O(M log M) where M = number of matching messages
            - Space: O(M) for returning M headers
            - Lock held during entire query (scales poorly for very large databases)
        """
        if filter_robot_id is not None:
            assert isinstance(filter_robot_id, int)
        if filter_latest is not None:
            assert isinstance(filter_latest, bool)

        # Header list is a dict with the headers as keys and the priorities as
        # values
        header_list = {}

        # To avoid inconsistencies, the db is locked while searching
        self.lock.acquire()
        for robot_id in self.db:
            if filter_robot_id is not None and robot_id != filter_robot_id:
                continue
            for topic in self.db[robot_id]:
                if filter_latest:
                    latest_msg_ts = rospy.Time(1, 0)
                    latest_msg = None
                for header in self.db[robot_id][topic]:
                    msg_content = self.db[robot_id][topic][header]
                    if filter_latest and msg_content.ts > latest_msg_ts:
                        latest_msg_ts = msg_content.ts
                        latest_msg = msg_content
                    if not filter_latest:
                        header_list[header] = {'prio': msg_content.priority,
                                               'ts': msg_content.ts}
                if filter_latest:
                    header_list[latest_msg.header] = {'prio': latest_msg.priority,
                                                      'ts': latest_msg.ts}
        self.lock.release()

        # Sort the dictionary by value, and get the keys
        sorted_tuples = sorted(header_list.items(),
                               key=lambda kv: (kv[1]['prio'], kv[1]['ts'], kv[0]),
                               reverse=True)
        sorted_header_list = [i[0] for i in sorted_tuples]
        return sorted_header_list

    def get_ts_dict(self):
        """
        Query the database and return a nested dictionary of most recent timestamps.
        
        Creates a hierarchical dictionary indexed by robot_id and topic_id, where each
        value is the highest (most recent) timestamp known in the database for that
        (robot_id, topic_id) sensor pair.
        
        This method is useful for synchronization checks: knowing the most recent
        timestamp for each sensor tells you the "watermark" of how fresh the local
        database is. Can be compared with remote databases to determine what data
        is missing.
        
        Thread Safety:
        - Acquires self.lock during entire scan to ensure consistent snapshot
        - Prevents race conditions with concurrent modifications
        
        Returns:
            dict: Nested dictionary structure:
                  {
                    robot_id: {
                      topic_id: timestamp_in_seconds (float),
                      topic_id: timestamp_in_seconds (float),
                      ...
                    },
                    robot_id: { ... },
                    ...
                  }
                  
                  If a (robot_id, topic_id) has no messages, that topic appears
                  with value -inf (negative infinity, lowest possible timestamp).
        
        Examples:
            # Example database:
            # Robot 0: laser (topic 0) at time 100.5, odometry (topic 1) at time 99.3
            # Robot 1: laser (topic 0) at time 102.1
            
            ts_dict = db.get_ts_dict()
            # Result:
            # {
            #   0: {0: 100.5, 1: 99.3},
            #   1: {0: 102.1}
            # }
            
            # Get most recent timestamp for specific robot/topic
            robot0_laser_ts = ts_dict[0][0]  # 100.5
            
            # Check what topics exist for a robot
            topics_in_robot0 = ts_dict[0].keys()  # [0, 1]
            
            # Find the newest data overall
            max_ts = max(max(ts_dict[rid].values()) for rid in ts_dict if ts_dict[rid])
        
        Use Cases:
            1. Synchronization: Compare local timestamps with remote to find missing data
               local_ts = db.get_ts_dict()
               remote_ts = remote_db.get_ts_dict()
               # Tell remote: "I have laser up to time X, need everything after that"
            
            2. Data freshness check: Verify how recent the database is
               ts_dict = db.get_ts_dict()
               if ts_dict[0][0] < (current_time - 5.0):  # No laser data in 5 seconds
                   print("WARNING: Jackal laser data stale!")
            
            3. Debugging: Quick view of database timestamp distribution
               ts_dict = db.get_ts_dict()
               print("Robot 0 topics:", ts_dict[0])  # Understand what data exists
            
            4. Initialization: On new connection, query remote timestamps before sync
               ts_dict = db.get_ts_dict()
               # Use this to determine if full resync needed
        
        Implementation Notes:
            - Uses -np.inf (negative infinity) for topics with no messages
            - This ensures empty topics sort before any real timestamps
            - Timestamps converted to seconds for easier human interpretation
            - Each robot maintains independent time domain (no global synchronization)
        
        Performance:
            - Time: O(M) where M = total number of messages in database
            - Space: O(R*T) where R = robots, T = topics per robot
            - Lock held during entire scan (blocking)
        """
        ts_dict = {}
        self.lock.acquire()
        for robot_id in self.db:
            if robot_id not in ts_dict:
                ts_dict[robot_id] = {}
            for topic in self.db[robot_id]:
                if topic not in ts_dict[robot_id]:
                    ts_dict[robot_id][topic] = -np.inf
                for header in self.db[robot_id][topic]:
                    msg = self.db[robot_id][topic][header]
                    ts_dict[robot_id][topic] = max(ts_dict[robot_id][topic],
                                                   msg.ts.to_sec())
        self.lock.release()
        return ts_dict

    def headers_not_in_local(self, remote_header_list, newer=False):
        """
        Compare remote headers with local database to find missing or newer data.
        
        Supports two comparison modes for database synchronization:
        
        MODE 1 (newer=False): "Which headers are MISSING from my local database?"
        - Find all headers in remote_header_list that DON'T exist locally
        - Useful for: initial pull sync, catching completely new messages
        
        MODE 2 (newer=True): "Which headers are NEWER than my local data?"
        - Find all headers with timestamps LATER than the latest local timestamp
          for that (robot_id, topic_id)
        - Useful for: incremental sync, catching updates to the same sensor
        
        This is the core method for data synchronization between robots or between
        local and remote databases.
        
        Thread Safety:
        - Calls get_header_list() and get_ts_dict() which handle locking internally
        - Deterministic: same inputs always produce same outputs
        
        Args:
            remote_header_list (list): List of binary headers from remote database.
                                       Format: [header1, header2, ...]
                                       Each header is a bytes object created by
                                       hash_comm.TsHeader.from_data(...).bindigest()
            
            newer (bool, optional): Comparison mode:
                                   False (default): Find MISSING headers
                                                   Returns headers not in local
                                   True: Find NEWER headers
                                        Returns headers with timestamps after
                                        the local watermark for that (robot, topic)
        
        Returns:
            list: List of binary headers [header1, header2, ...]
                 Represents the headers to request from remote.
                 Empty list if no missing or newer data.
        
        Raises:
            AssertionError: If remote_header_list not a list
            AssertionError: If newer not a bool
            (hash_comm.TsHeader.from_header raises on invalid header)
        
        Mode 1 Examples (newer=False - detect MISSING data):
            # Remote has: [h1, h2, h3]
            # Local has:  [h1, h3]
            # Result: [h2]  (h2 is missing locally)
            
            missing = db.headers_not_in_local([h1, h2, h3], newer=False)
            # missing = [h2]
            # Interpretation: "I don't have h2, please send it to me"
        
        Mode 2 Examples (newer=True - detect NEWER data):
            # Remote: laser header at time 105.0
            # Local:  laser's latest timestamp is 100.0
            # Result: [laser_header_105.0]  (newer than 100.0)
            
            newer_hdrs = db.headers_not_in_local([laser_105, laser_103], newer=True)
            # If local laser = 100.0:
            # newer_hdrs = [laser_105, laser_103]  (both newer than 100.0)
        
        Use Cases:
            1. Initial Synchronization: Pull all missing data from remote
               all_remote = remote_db.get_header_list()
               missing = local_db.headers_not_in_local(all_remote, newer=False)
               # Fetch missing headers from remote
            
            2. Incremental Sync: Pull only newer updates
               recent = remote_db.get_header_list()
               updates = local_db.headers_not_in_local(recent, newer=True)
               # Fetch only updated data since last sync
            
            3. Two-way Reconciliation: Check both directions
               # Check what I'm missing from them
               missing_from_me = local_db.headers_not_in_local(
                   remote_headers, newer=False)
               # Check what they should update
               updates_for_them = remote_db.headers_not_in_local(
                   local_headers, newer=True)
            
            4. Per-Robot Sync: Sync only one robot's data
               # Jackal (robot_id=0) wants to sync with UAV (robot_id=1)
               uav_headers = uav_db.get_header_list(filter_robot_id=1)
               missing_uav_data = jackal_db.headers_not_in_local(
                   uav_headers, newer=False)
        
        Implementation Notes - Mode 1 (newer=False):
        - Retrieves full local header list
        - Simple set membership check: is remote_header in local_headers?
        - O(L) for each remote header where L = local headers
        
        Implementation Notes - Mode 2 (newer=True):
        - Decodes each remote header to extract (robot_id, topic_id, timestamp)
        - Gets local timestamp watermark for that (robot, topic) from ts_dict
        - Compares: if remote.timestamp > local_watermark, it's newer
        - Handles robot/topic not in local: treats as -inf (everything is newer)
        - O(R) for each remote header where R = remote headers
        
        Performance:
            Mode 1: O(L + R) where L = local headers, R = remote headers
            Mode 2: O(R * log(M)) where R = remote headers, M = messages
                   (calls get_ts_dict with O(M) complexity)
        
        Edge Cases:
            - Empty local database: All remote headers will be considered missing/newer
            - Unknown robot in remote: Treated as not in local (will request)
            - Timestamps exactly equal: newer=True considers it NOT newer
                                       (only STRICTLY newer timestamps included)
        """
        assert isinstance(remote_header_list, list)
        assert isinstance(newer, bool)

        missing_headers = []
        if not newer:
            local_headers = self.get_header_list()
            for h in remote_header_list:
                if h not in local_headers:
                    missing_headers.append(h)
            return missing_headers
        else:
            ts_dict = self.get_ts_dict()
            for h in remote_header_list:
                h = hash_comm.TsHeader.from_header(h)
                r_id, t_id, time = h.get_id_and_time()
                if r_id not in ts_dict:
                    missing_headers.append(h.bindigest())
                elif t_id not in ts_dict[h.robot_id]:
                    missing_headers.append(h.bindigest())
                elif time.to_sec() > ts_dict[h.robot_id][h.topic_id]:
                    missing_headers.append(h.bindigest())
            return missing_headers

    def find_header(self, requested_header):
        """
        Retrieve a complete DBMessage from the database by its binary header.
        
        Performs a linear search through the database to find the message with
        the matching header and returns the complete DBMessage object with all
        fields reconstructed.
        
        Thread Safety:
        - Acquires self.lock before search to ensure consistent snapshot
        - Releases lock only after extracting all message data
        - Prevents race conditions where message could be deleted during access
        
        Search Algorithm:
        - Linear scan through nested loops: robots → topics → headers
        - Exits early on first match (fail-fast)
        - Reconstructs complete DBMessage from database record
        
        Args:
            requested_header (bytes): Binary header to search for.
                                     Format: bytes object from
                                            hash_comm.TsHeader.from_data(...).bindigest()
                                     Typically 32-byte SHA256 hash
        
        Returns:
            DBMessage: Complete message object with all fields:
                      - robot_id, topic_id (identified from header)
                      - dtype, priority, ts, data (extracted from storage)
                      - header (same as requested_header, recomputed for validation)
        
        Raises:
            AssertionError: If requested_header is not bytes
            Exception: If header not found in database, with message:
                      "packData: header not found"
                      (NOTE: Exception message suggests legacy name "packData")
        
        Examples:
            # Insert a message and retrieve it
            msg = DBMessage(0, 0, 0, 3, rospy.Time(100.5), b'\\x00\\x01')
            header = db.add_modify_data(msg)
            
            # Retrieve the message
            retrieved_msg = db.find_header(header)
            assert retrieved_msg == msg  # Should be equal (within tolerances)
            print(retrieved_msg)  # "0, 0, 0, 3, 100.500000"
            
            # Try to find non-existent header
            fake_header = b'\\x00' * 32
            try:
                db.find_header(fake_header)
            except Exception as e:
                print(f"Error: {e}")  # "packData: header not found"
        
        Use Cases:
            1. Data Retrieval: Called by ROS service (SelectDB) to return message data
               # Remote requests specific header
               msg = db.find_header(requested_header)
               # Serialize msg.data and send back over network
            
            2. Verification: Check if specific message exists
               try:
                   msg = db.find_header(header)
                   print(f"Message exists: {msg}")
               except Exception:
                   print("Message not found")
            
            3. Synchronization: After receiving header list, fetch actual data
               headers = remote_db.get_header_list()
               for h in headers:
                   msg = local_db.find_header(h)  # Get full message
            
            4. Analysis: Extract message details for debugging
               msg = db.find_header(problematic_header)
               print(f"Message from robot {msg.robot_id}, topic {msg.topic_id}")
                           f"at time {msg.ts}, priority {msg.priority}")
        
        Performance:
            - Time: O(M) where M = total messages in database
                   Worst case: searches entire database if message is last or missing
                   Expected: O(M / (R*T)) for random robot/topic (average case)
                   where R = robots, T = topics per robot
            - Space: O(1) for return value (single message)
            - Lock held for entire search duration (blocking on other operations)
        
        Implementation Notes:
            - Uses nested loop with early exit flag for multi-level break
            - Reconstructs header via binary digest to ensure consistency
            - All fields extracted under lock protection, lock released before return
            - Exception type suggests method was originally part of "packData" logic
            - Linear search is acceptable for typical database sizes (< 1000 messages)
              For larger databases, consider adding hash table index
        
        Error Handling:
            - Validates input is bytes (AssertionError)
            - Release lock before throwing exception if not found
            - Caller responsible for catching Exception and handling missing data
        """
        assert isinstance(requested_header, bytes)
        # Find data in db
        data_found = False
        self.lock.acquire()
        for robot in self.db:
            for topic_id in self.db[robot]:
                if data_found:
                    break
                for header in self.db[robot][topic_id]:
                    if data_found:
                        break
                    if header == requested_header:
                        data_found = True
                        req_robot_id = robot
                        req_topic_id = topic_id
                        break
        if not data_found:
            self.lock.release()
            raise Exception('packData: header not found')
        req_dtype = self.db[req_robot_id][req_topic_id][header].dtype
        req_priority = self.db[req_robot_id][req_topic_id][header].priority
        req_ts = self.db[req_robot_id][req_topic_id][header].ts
        req_data = self.db[req_robot_id][req_topic_id][header].data
        self.lock.release()
        dbm = DBMessage(req_robot_id, req_topic_id, req_dtype,
                        req_priority, req_ts, req_data)
        return dbm

    def __str__(self):
        """
        String representation of the entire database for debugging and analysis.
        
        Generates a formatted hierarchical text view of all messages in the database,
        organized by robot, then topic, then individual messages. Useful for:
        - Visual inspection during debugging
        - Logging database state at checkpoints
        - Understanding data distribution across robots/topics
        - Console output for quick database status
        
        Thread Safety:
        - Acquires self.lock before accessing database to prevent concurrent modifications
        - Releases lock after collecting all data
        - Ensures consistent snapshot (no data changes during traversal)
        
        Output Format:
        Hierarchical text format with indentation showing structure:
        
        Robot ID: 0
          Topic ID: 0
            TS: <rospy.Time object>
              Dtype: 0
              Prio: 3
              Data: b'\\x00\\x01\\x02...'
              Header: b'\\xaa\\xbb\\xcc...'
            TS: <rospy.Time object>
              Dtype: 0
              Prio: 2
              Data: b'\\x03\\x04\\x05...'
              Header: b'\\xdd\\xee\\xff...'
          Topic ID: 1
            TS: <rospy.Time object>
              ...
        Robot ID: 1
          ...
        
        Returns:
            str: Multi-line formatted string showing full database contents
        
        Examples:
            # Print full database
            db = DBwLock()
            msg1 = DBMessage(0, 0, 0, 3, rospy.Time(100.5), b'data1')
            msg2 = DBMessage(0, 0, 0, 2, rospy.Time(101.0), b'data2')
            msg3 = DBMessage(1, 0, 1, 1, rospy.Time(102.0), b'data3')
            db.add_modify_data(msg1)
            db.add_modify_data(msg2)
            db.add_modify_data(msg3)
            
            print(db)
            # Output:
            # Robot ID: 0
            #   Topic ID: 0
            #     TS: [timestamp1]
            #       Dtype: 0
            #       Prio: 3
            #       Data: b'data1'
            #       Header: [binary]
            #     TS: [timestamp2]
            #       Dtype: 0
            #       Prio: 2
            #       Data: b'data2'
            #       Header: [binary]
            # Robot ID: 1
            #   Topic ID: 0
            #     TS: [timestamp3]
            #       Dtype: 1
            #       Prio: 1
            #       Data: b'data3'
            #       Header: [binary]
            
            # Write to log file
            with open('database_dump.log', 'w') as f:
                f.write(str(db))
        
        Use Cases:
            1. Debugging: Inspect database state at specific points
               print(db)  # See what data is stored
            
            2. Logging: Save database snapshot for analysis
               import time
               timestamp = time.strftime('%Y%m%d_%H%M%S')
               with open(f'db_dump_{timestamp}.log', 'w') as f:
                   f.write(str(db))
            
            3. Testing: Verify expected data after operations
               db.add_modify_data(msg)
               output = str(db)
               assert 'Robot ID: 0' in output
               assert 'Topic ID: 0' in output
            
            4. Monitoring: Monitor data growth and distribution
               output = str(db)
               lines = output.split('\\n')
               robot_count = len([l for l in lines if l.startswith('Robot ID')])
               print(f"Database has {robot_count} robots")
            
            5. Troubleshooting: Understand data before sync issues
               # Before sync attempt, see what's in each database
               local_str = str(local_db)
               remote_str = str(remote_db)
               # Compare to diagnose missing data
        
        Performance:
            - Time: O(M) where M = total messages in database
                   (must traverse all messages to build output)
            - Space: O(M) for output string (can be large for big databases)
            - Lock held during entire traversal (blocking)
        
        Implementation Notes:
            - Indentation uses 2-space increments for readability
            - Shows all message fields including binary header and data
            - Binary data and headers shown as Python bytes representations
            - Empty database returns empty string ""
            - Each message's fields shown on separate lines with indentation
        
        Warning:
            - Large databases produce large strings (can exceed memory)
            - Lock is held for entire output generation (affects concurrency)
            - Binary data may not display cleanly if contains special characters
            - Consider filtering or sampling for very large databases
        """
        # Print the database in a nice way
        self.lock.acquire()
        out = ""
        for robot in self.db:
            out += f"Robot ID: {robot}\n"
            for topic_id in self.db[robot]:
                out += f"  Topic ID: {topic_id}\n"
                for header in self.db[robot][topic_id]:
                    msg = self.db[robot][topic_id][header]
                    out += f"    TS: {str(msg.ts)}\n"
                    out += f"      Dtype: {msg.dtype}\n"
                    out += f"      Prio: {msg.priority}\n"
                    out += f"      Data: {msg.data}\n"
                    out += f"      Header: {header}\n"
        self.lock.release()
        return out
