#!/usr/bin/env python3
"""
HASH COMMUNICATION MODULE
========================
Binary header creation and hashing utilities for the MOCHA multi-robot framework.

This module provides two main components:

1. Hash Class: Creates truncated SHA256 hashes (6 bytes = 48 bits)
   - Used for various data hashing operations
   - Truncation provides good balance of collision resistance vs. space efficiency
   - Birthday paradox collision probability: ~16.7 million random hashes before collision
   - Reference: https://preshing.com/20110504/hash-collision-probabilities/

2. TsHeader Class: Binary headers for multi-robot message identification
   - Encodes: robot_id (1 byte), topic_id (1 byte), timestamp (4 bytes)
   - Total: 6 bytes (48 bits) - fits efficiently in network packets
   - Deterministic encoding: Same (robot, topic, time) always produces same header
   - Used as primary key for database message lookups

Binary Format of TsHeader (6 bytes total):
  Bytes 0:1  - Robot ID (0-255, 1 byte unsigned)
  Bytes 1:2  - Topic ID (0-255, 1 byte unsigned)  
  Bytes 2:4  - Seconds (0-65535, 2 bytes unsigned, modulo 65536)
  Bytes 4:6  - Milliseconds (0-999, 2 bytes unsigned)
  
  Network byte order (Big-Endian): struct format "!BBHH"
  Example: robot=0, topic=0, time=100.5s
    → [0x00, 0x00, 0x00, 0x64, 0x01, 0xF4]

Time Resolution:
  - Seconds: 16-bit (0-65535) with wraparound at 65536 (~18.2 hours)
  - Subsecond: Nanoseconds converted to milliseconds (3 decimal places precision)
  - Total granularity: 1 millisecond (meets ROS sync requirements)
  - Note: Precision loss = ~1 millisecond (acceptable for sensor data)

Dependencies:
  - hashlib: SHA256 cryptographic hash function
  - rospy: ROS Python interface for Time objects
  - struct: Binary data packing/unpacking (network byte order)

Use Cases:
  - Identifying messages uniquely across multi-robot systems
  - Creating consistent headers for network communication
  - Hashing arbitrary data for deduplication
  - Synchronization between robots (deterministic headers)
"""

import hashlib
import rospy
import struct

LENGTH = 6


class Hash():
    """
    Truncated SHA256 Hash Generator
    ================================
    Creates 6-byte (48-bit) SHA256 hashes from binary data.
    
    The 6-byte truncation provides:
    - Space efficiency: ~18x smaller than full SHA256 (32 bytes)
    - Collision resistance: ~16.7 million items before 50% collision risk
    - Network efficiency: Fits in minimal packet overhead
    - Cryptographic security: Maintains SHA256 properties for truncated bits
    
    Class Attributes:
        HASH_LENGTH (int): Length of hash in bytes (always 6)
        HASH_LENGTH_BITS (int): Length of hash in bits (always 48)
    
    Use Cases:
        - Checksums for data deduplication
        - Content-addressable storage keys
        - Quick fingerprints for large data
        - Message integrity verification
    
    Limitation:
        - Truncation reduces collision resistance compared to full SHA256
        - For cryptographic applications, use full SHA256 instead
        - For MOCHA, 48-bit hash is sufficient for typical multi-robot data volumes
    """
    HASH_LENGTH = LENGTH
    # Length of the hash in bits
    # We can expect a collision after approx
    # math.sqrt(2**HASH_LENGTH_BITS)
    # https://preshing.com/20110504/hash-collision-probabilities/
    HASH_LENGTH_BITS = HASH_LENGTH*8

    def __init__(self, data):
        """
        Initialize Hash object with data to hash.
        
        Args:
            data (bytes): Binary data to hash. Must be bytes object.
                         String data must be encoded: "text".encode()
        
        Raises:
            TypeError: If data is not bytes type
        
        Examples:
            # Hash binary data
            h = Hash(b'\x00\x01\x02')
            digest = h.digest()  # 6-byte hash
            
            # Hash encoded string
            h = Hash("sensor_data".encode())
            digest = h.digest()
            
            # Wrong: passing string directly
            try:
                h = Hash("string")  # TypeError!
            except TypeError:
                print("Must encode string to bytes")
        """
        if type(data) != bytes:
            raise TypeError
        self.data = data

    def digest(self):
        """
        Compute 6-byte truncated SHA256 hash of the stored data.
        
        Process:
        1. Compute full SHA256 hash (32 bytes) of self.data
        2. Truncate to first 6 bytes
        3. Return as bytes object
        
        Returns:
            bytes: 6-byte (48-bit) SHA256 hash
                  Deterministic: same input always produces same output
                  Binary format: can contain any byte values 0x00-0xFF
        
        Examples:
            # Hash computation
            h = Hash(b'test_data')
            digest = h.digest()
            print(len(digest))  # 6
            print(type(digest))  # <class 'bytes'>
            
            # Deterministic property
            h1 = Hash(b'same').digest()
            h2 = Hash(b'same').digest()
            assert h1 == h2  # Always identical
            
            # Different inputs produce different hashes
            h1 = Hash(b'data1').digest()
            h2 = Hash(b'data2').digest()
            assert h1 != h2  # Very unlikely to collide
        
        Use Cases:
            - Create checksums for data integrity
            - Generate keys for content-addressable storage
            - Quick fingerprints for deduplication
            - Network message identification
        
        Performance:
            - Time: O(n) where n = len(data) (SHA256 overhead)
            - Space: O(1) - returns 6-byte hash
            - Typically < 1ms for data < 1MB
        
        Notes:
            - SHA256 is cryptographically secure but truncation reduces strength
            - 48-bit hash has collision risk: ~16.7M items before 50% collision
            - For cryptographic security, use full SHA256
            - For MOCHA, 48-bit is sufficient for typical message volumes
        """
        h = hashlib.sha256(self.data)
        hexdigest = h.digest()
        trimmed_digest = hexdigest[:self.HASH_LENGTH]
        return trimmed_digest

    def bindigest(self):
        """
        Compute hash (alias for digest()).
        
        Provided for naming consistency with TsHeader.bindigest().
        Both classes have bindigest() method for uniform interface.
        
        Returns:
            bytes: Same as digest() - 6-byte truncated SHA256 hash
        
        Examples:
            h = Hash(b'data')
            digest1 = h.digest()
            digest2 = h.bindigest()
            assert digest1 == digest2  # Identical
        """
        return self.digest()


class TsHeader():
    """
    Binary Header for Multi-Robot Message Identification
    =====================================================
    Encodes (robot_id, topic_id, timestamp) into a 6-byte binary header.
    
    This class is the primary mechanism for creating consistent, deterministic
    message identifiers across the MOCHA framework. Each message is uniquely
    identified by a binary header derived from its source (robot/topic) and
    timestamp.
    
    Binary Format (6 bytes, network byte order):
      Bytes 0:1   - robot_id (unsigned 8-bit, 0-255)
      Bytes 1:2   - topic_id (unsigned 8-bit, 0-255)
      Bytes 2:4   - secs (unsigned 16-bit, 0-65535, modulo 65536)
      Bytes 4:6   - msecs (unsigned 16-bit, 0-999 milliseconds)
    
    Time Resolution:
      - Seconds: 16-bit with wraparound every 65536 seconds (~18.2 hours)
      - Milliseconds: 1000ms = 1 second, so precision = 1 millisecond
      - Original ROS nanoseconds converted to milliseconds (1ms granularity)
    
    Determinism:
      - Same (robot_id, topic_id, timestamp) always produces same header
      - Headers are reproducible across processes and machines
      - Enables consistent message identification for synchronization
    
    Usage Pattern:
      CREATION: Use from_data() to create from ROS/Python objects
      SERIALIZATION: Use bindigest() to get 6-byte binary form
      DESERIALIZATION: Use from_header() to reconstruct from bytes
      RETRIEVAL: Use get_id_and_time() to extract components
    
    Class Attributes:
        HEADER_LENGTH (int): Size of binary header (6 bytes)
    
    Instance Attributes:
        robot_id (int): Source robot ID (0-255)
        topic_id (int): Source topic/sensor ID (0-255)
        secs (int): Seconds component (0-65535, wraps)
        msecs (int): Milliseconds component (0-999)
    
    Construction Methods:
        - from_data(robot_id, topic_id, time): Create from ROS objects
        - from_header(header): Reconstruct from 6-byte binary
        - __init__(**kwargs): Internal only (use from_data or from_header)
    """
    # Get length in bytes
    HEADER_LENGTH = LENGTH

    def __init__(self, *, robot_id=None, topic_id=None, secs=None, msecs=None):
        """
        Internal constructor for TsHeader.
        
        WARNING: Do NOT use this constructor directly!
        Use from_data() or from_header() instead.
        
        This method is for internal use only. Using __init__ directly bypasses
        all validation and can create invalid headers.
        
        Args:
            robot_id (int, optional): Robot ID (0-255)
            topic_id (int, optional): Topic ID (0-255)
            secs (int, optional): Seconds (0-65535)
            msecs (int, optional): Milliseconds (0-999)
        
        Attributes:
            Stores all four components for later binary packing or retrieval
        
        Important:
            For creating headers from ROS data:
                header = TsHeader.from_data(robot_id, topic_id, rospy_time)
            
            For deserializing 6-byte binary:
                header = TsHeader.from_header(binary_bytes)
            
            For manual construction (internal only):
                obj = TsHeader(robot_id=0, topic_id=0, secs=100, msecs=500)
        """
        # You should not use this constructor to create the class. Use
        # from_data or from_header instead
        self.robot_id = robot_id
        self.topic_id = topic_id
        self.secs = secs
        self.msecs = msecs

    @classmethod
    def from_data(cls, robot_id, topic_id, time):
        """
        Create TsHeader from ROS/Python objects (PREFERRED METHOD).
        
        Converts Python integers and ROS Time objects into a TsHeader instance,
        with full validation of input values. This is the primary method for
        creating headers in normal operation.
        
        Validation:
        1. robot_id must be int in range [0, 255] (1 byte unsigned)
        2. topic_id must be int in range [0, 255] (1 byte unsigned)
        3. time must be rospy.Time object with non-negative secs and nsecs
        
        Time Conversion:
        - rospy.Time.secs → secs % 65536 (wrap at 18.2 hours)
        - rospy.Time.nsecs → msecs = nsecs // 1,000,000 (nanosec → millisec)
        - Result: 1 millisecond precision, timestamp wraps every ~18.2 hours
        
        Args:
            robot_id (int): Source robot identifier (0-255)
                           0 = Jackal UGV, 1 = Hector Quadrotor UAV
            topic_id (int): Source sensor/topic identifier (0-255)
                           0 = laser, 1 = camera, 2 = gps, etc.
            time (rospy.Time): ROS timestamp of message creation
                              rospy.Time(secs=100, nsecs=500000000)
                              = 100.5 seconds since epoch
        
        Returns:
            TsHeader: Initialized object with validated and converted components
                     Ready for binary serialization via bindigest()
        
        Raises:
            AssertionError: If any parameter fails validation:
                - robot_id not int or out of range [0, 255]
                - topic_id not int or out of range [0, 255]
                - time not rospy.Time object
                - time.secs or time.nsecs negative
        
        Examples:
            # Create header from ROS objects
            header = TsHeader.from_data(
                robot_id=0,
                topic_id=0,
                time=rospy.Time(100.5)  # 100 seconds, 500 ms
            )
            
            # Serialize to 6-byte binary
            binary = header.bindigest()
            print(len(binary))  # 6 bytes
            
            # Use in database reference
            msg = DBMessage(0, 0, 0, 3, times, data)
            header = TsHeader.from_data(0, 0, times)
            # (DBMessage automatically creates header internally)
            
            # For Hector Quadrotor UAV (robot_id=1) odometry (topic_id=1)
            uav_header = TsHeader.from_data(1, 1, rospy.Time.now())
        
        Use Cases:
            1. Create message identifiers during publishing
               msg = DBMessage(robot_id, topic_id, dtype, prio, ts, data)
               # DBMessage internally calls TsHeader.from_data()
            
            2. Manually create headers for database queries
               header_obj = TsHeader.from_data(0, 0, query_time)
               header_binary = header_obj.bindigest()
            
            3. Test header creation with known values
               h = TsHeader.from_data(0, 0, rospy.Time(100, 0))
               # h.secs = 100, h.msecs = 0
        
        Time Resolution Details:
            Input:  rospy.Time(secs=100, nsecs=500,000,000)
            Output: secs=100, msecs=500
            Precision: 1 millisecond
            
            Input:  rospy.Time(secs=100, nsecs=123,456,789)
            Output: secs=100, msecs=123
            Loss: 456,789 nanoseconds truncated to milliseconds
        
        Wraparound Behavior:
            Input:  rospy.Time(secs=65600, nsecs=0)
            Output: secs = 65600 % 65536 = 64
            Note: Time wraps every 65536 seconds (~18.2 hours)
                 This is acceptable for local robot sessions
                 For multi-day operation, handle wraparound in application
        
        Performance:
            - Time: O(1) constant time
            - Space: O(1) single object creation
            - Validation: ~5 assertions checked
        
        Implementation Notes:
            - Redundant assignment: robot_id = robot_id (kept for clarity)
            - Modulo 65536 prevents overflow in 16-bit field
            - Integer division // ensures integer milliseconds (no floats)
            - All inputs validated immediately (fail-fast approach)
        """
        # Robot ID is a 1-byte unsigned integer. Check type and value
        assert type(robot_id) == int and robot_id >= 0 and robot_id < 256
        # Same for topic ID
        assert type(topic_id) == int and topic_id >= 0 and topic_id < 256
        # time is a rospy time object
        assert type(time) == rospy.Time

        # The header will be comprised of 1 byte for the robot number, 1 byte
        # for the topic id, and 4 bytes for the time. The first 2 bytes of the
        # time are the secs, and the 2 last bytes are the ms of the message.
        # This means that we have ms resolution. We only handle positive times
        assert type(time.secs) == int and time.secs >= 0
        assert type(time.nsecs) == int and time.nsecs >= 0
        secs = time.secs % 65536
        msecs = time.nsecs // 1000000
        robot_id = robot_id
        topic_id = topic_id
        return cls(robot_id=robot_id, topic_id=topic_id,
                   secs=secs, msecs=msecs)

    @classmethod
    def from_header(cls, header):
        """
        Reconstruct TsHeader from 6-byte binary form (DESERIALIZATION).
        
        Reverses the binary serialization process. Given a 6-byte binary header
        (previously created by bindigest()), reconstructs all components.
        
        Binary Format (6 bytes, network byte order):
          Bytes 0:1 - robot_id (unsigned byte, format "!B")
          Bytes 1:2 - topic_id (unsigned byte, format "!B")
          Bytes 2:4 - secs (unsigned short, format "!H")
          Bytes 4:6 - msecs (unsigned short, format "!H")
        
        Args:
            header (bytes): 6-byte binary header in network byte order
                           Must be exactly 6 bytes
                           Typically created by TsHeader.from_data(...).bindigest()
        
        Returns:
            TsHeader: Reconstructed object with extracted components
                     Ready for use or re-serialization
        
        Raises:
            struct.error: If header is not exactly 6 bytes
            IndexError: If header is shorter than expected bytes
        
        Examples:
            # Round-trip serialization
            original = TsHeader.from_data(0, 0, rospy.Time(100.5))
            binary = original.bindigest()
            reconstructed = TsHeader.from_header(binary)
            
            # Verify components match
            assert reconstructed.robot_id == 0
            assert reconstructed.topic_id == 0
            assert reconstructed.secs == 100
            assert reconstructed.msecs == 500
            
            # Use reconstructed header to get components
            robot_id, topic_id, time = reconstructed.get_id_and_time()
        
        Use Cases:
            1. Deserialize headers received over network
               remote_header_bytes = network_receive(6)
               header = TsHeader.from_header(remote_header_bytes)
            
            2. Reconstruct headers from database storage
               stored_bytes = db[robot_id][topic_id][header_key]
               header = TsHeader.from_header(stored_bytes)
            
            3. Verify round-trip serialization
               h1 = TsHeader.from_data(...)
               h2 = TsHeader.from_header(h1.bindigest())
               assert h1 == h2  # Components match
            
            4. Parse headers in synchronization protocol
               incoming_headers = [b'...', b'...', b'...']  # 6-byte each
               parsed = [TsHeader.from_header(h) for h in incoming_headers]
        
        Binary Format Example:
            Input:  b'\\x00\\x00\\x00\\x64\\x01\\xF4'
            Bytes:  [robot=0, topic=0, secs=100, msecs=500]
            Result: TsHeader(robot_id=0, topic_id=0, secs=100, msecs=500)
        
        Struct Unpacking:
            "!B" - Network byte order (!), unsigned byte (B)
            "!H" - Network byte order (!), unsigned short (H, 2 bytes)
        
        Performance:
            - Time: O(1) constant time (6 unpack operations)
            - Space: O(1) single object creation
            - No validation (assumes valid binary input)
        
        Important:
            - No validation of ranges (assumes binary is valid)
            - For validation, pass through from_data() instead
            - Assumes input is exactly 6 bytes (will raise IndexError if not)
            - Network byte order enforces platform-independent serialization
        """
        robot_id = struct.unpack("!B", header[0:1])[0]
        topic_id = struct.unpack("!B", header[1:2])[0]
        secs = struct.unpack("!H", header[2:4])[0]
        msecs = struct.unpack("!H", header[4:6])[0]
        return cls(robot_id=robot_id, topic_id=topic_id,
                   secs=secs, msecs=msecs)

    def bindigest(self):
        """
        Serialize TsHeader to 6-byte binary (PRIMARY SERIALIZATION).
        
        Converts robot_id, topic_id, secs, and msecs into a packed 6-byte
        binary representation suitable for storage or network transmission.
        
        This is the primary method for serializing headers. The resulting binary
        is deterministic (same input always produces same output) and can be
        transmitted over the network or stored in a database.
        
        Binary Format (6 bytes, network byte order):
          Bytes 0:1 - robot_id (unsigned byte, "!B")
          Bytes 1:2 - topic_id (unsigned byte, "!B")
          Bytes 2:4 - secs (unsigned short, "!H")
          Bytes 4:6 - msecs (unsigned short, "!H")
        
        Returns:
            bytes: 6-byte binary serialization in network byte order
                  Use from_header() to deserialize back to TsHeader
        
        Raises:
            struct.error: If any component is out of valid range
        
        Examples:
            # Create and serialize
            header = TsHeader.from_data(0, 0, rospy.Time(100.5))
            binary = header.bindigest()
            print(len(binary))  # 6
            print(type(binary))  # <class 'bytes'>
            print(binary)  # b'\\x00\\x00\\x00\\x64\\x01\\xf4'
            
            # Use as message key in database
            db[robot_id][topic_id][binary] = message
            
            # Transmit over network
            socket.send(binary)
            
            # Store in file
            with open('header', 'wb') as f:
                f.write(binary)
        
        Reverse Operation:
            binary = header.bindigest()
            reconstructed = TsHeader.from_header(binary)
            # All components match original
        
        Network Byte Order:
            "!" prefix ensures network byte order (Big-Endian)
            Ensures cross-platform compatibility:
            - Intel x86 (Little-Endian) ↔ ARM Big-Endian
            - Client ↔ Server communication
            - File format persistence
        
        Use Cases:
            1. Create database keys
               header = TsHeader.from_data(robot_id, topic_id, time)
               db_key = header.bindigest()
               db[robot_id][topic_id][db_key] = message_obj
            
            2. Network transmission
               # Send header to remote robot
               socket.send(header.bindigest())
            
            3. Create message headers for ROS
               # Store as message ID for logging/tracking
               msg_id = header.bindigest()
            
            4. File/database storage
               # Persist headers to disk
               try:
                   with open('headers.bin', 'ab') as f:
                       f.write(header.bindigest())
            
            5. Header comparison (binary form)
               h1 = TsHeader.from_data(...)
               h2 = TsHeader.from_data(...)
               if h1.bindigest() == h2.bindigest():
                   print("Same message")
        
        Struct Packing:
            struct.pack("!B", robot_id)  # 1 byte
            struct.pack("!B", topic_id)  # 1 byte
            struct.pack("!H", secs)      # 2 bytes
            struct.pack("!H", msecs)     # 2 bytes
            Total: 6 bytes
        
        Binary Example:
            Input: TsHeader(robot_id=0, topic_id=0, secs=100, msecs=500)
            Output: b'\\x00\\x00\\x00\\x64\\x01\\xf4'
            Breakdown:
              - 0x00: robot_id = 0
              - 0x00: topic_id = 0
              - 0x00, 0x64: secs = 100 (big-endian short)
              - 0x01, 0xf4: msecs = 500 (big-endian short)
        
        Performance:
            - Time: O(1) constant (4 struct.pack calls)
            - Space: O(1) creates 6-byte object
            - Deterministic: same input always same output
        
        Important:
            - Must call set all four fields (robot_id, topic_id, secs, msecs)
            - struct.pack will raise if values out of range
            - Bytes 0-1: unsigned 8-bit (0-255 only!)
            - Bytes 2-3: unsigned 16-bit (0-65535 only!)
            - Network byte order enforces consistency across platforms
        """
        # Assemble the header by concatenating data, using struct
        b = struct.pack("!B", self.robot_id)
        b += struct.pack("!B", self.topic_id)
        b += struct.pack("!H", self.secs)
        b += struct.pack("!H", self.msecs)
        return b

    def get_id_and_time(self):
        """
        Extract robot_id, topic_id, and reconstructed rospy.Time from header.
        
        Reconstructs the three key components from a TsHeader object. This method
        reverses the conversions made during from_data(). Useful for deserializing
        a header and accessing its components without manually unpacking.
        
        Process:
        1. Verify all four fields are set (robot_id, topic_id, secs, msecs)
        2. Reconstruct rospy.Time from secs and msecs
           - Seconds: used directly
           - Milliseconds: converted back to nanoseconds (msecs * 1,000,000)
        3. Return tuple of (robot_id, topic_id, time)
        
        Returns:
            tuple: (robot_id, topic_id, rospy.Time)
                  robot_id (int): Source robot ID (0-255)
                  topic_id (int): Source topic/sensor ID (0-255)
                  rospy.Time: Reconstructed timestamp with millisecond precision
        
        Raises:
            AssertionError: If any field (robot_id, topic_id, secs, msecs) is None
        
        Examples:
            # Basic usage
            header = TsHeader.from_data(0, 0, rospy.Time(100.5))
            robot_id, topic_id, time = header.get_id_and_time()
            assert robot_id == 0
            assert topic_id == 0
            assert time.secs == 100
            assert time.nsecs == 500000000  # 500 ms = 500,000,000 ns
            
            # After deserialization
            binary = TsHeader.from_data(1, 2, rospy.Time(200.123)).bindigest()
            header = TsHeader.from_header(binary)
            rid, tid, ts = header.get_id_and_time()
            assert rid == 1
            assert tid == 2
            assert ts.secs == 200
            assert ts.nsecs == 123000000  # 123 ms precision
            
            # Use in message identification
            msg = db.find_header(some_header)
            robot_id, topic_id, timestamp = msg_header.get_id_and_time()
            print(f"Message from robot {robot_id}, topic {topic_id} at {timestamp}")
        
        Time Reconstruction:
            Input:  secs=100, msecs=500
            Output: rospy.Time(secs=100, nsecs=500_000_000)
            
            Input:  secs=100, msecs=123
            Output: rospy.Time(secs=100, nsecs=123_000_000)
            
            Note: Precision limited to milliseconds (last 3 nanosecond digits = 0)
        
        Use Cases:
            1. Extract header components for logging
               header = TsHeader.from_header(db_key)
               rid, tid, ts = header.get_id_and_time()
               print(f"[{rid}:{tid}] {ts}")  # "[0:5] 100.500000"
            
            2. Reconstruct message timestamp
               # After retrieving message by header
               msg = db.find_header(header_bytes)
               _, _, original_time = header_obj.get_id_and_time()
               # Use original_time for ROS pub/sub
            
            3. Filter messages by robot/topic
               for header_bytes in message_list:
                   header = TsHeader.from_header(header_bytes)
                   rid, tid, ts = header.get_id_and_time()
                   if rid == 0:  # Only Jackal messages
                       process_message(header_bytes)
            
            4. Create ROS messages with original timestamps
               header = TsHeader.from_header(incoming_header)
               rid, tid, ts = header.get_id_and_time()
               # Use ts as message timestamp in ROS publisher
               msg.header.stamp = ts
            
            5. Synchronization timing
               local_header = TsHeader.from_data(0, 0, rospy.Time.now())
               remote_header = TsHeader.from_header(network_data)
               _, _, remote_time = remote_header.get_id_and_time()
               time_diff = rospy.Time.now() - remote_time
        
        Component Mapping:
            TsHeader fields → Return values:
            self.robot_id → robot_id (returned unchanged)
            self.topic_id → topic_id (returned unchanged)
            self.secs → time.secs (returned as-is)
            self.msecs → time.nsecs (msecs * 1,000,000)
        
        Validation:
            All four fields are checked for None (AssertionError if any missing)
            This ensures header was properly initialized
        
        Performance:
            - Time: O(1) constant (4 assertions, 1 tuple creation)
            - Space: O(1) creates single rospy.Time object
        
        Important:
            - Precision limited to milliseconds (nanoseconds truncated)
            - Can only call after header properly initialized
            - rospy.Time.secs used directly (no wraparound conversion)
            - Inverse of from_data() but note time loss at millisecond level
        
        Precision Loss Example:
            Original: rospy.Time(100, 500_123_456)  # 500.123456 ms
            Stored:   secs=100, msecs=500
            Retrieved: rospy.Time(100, 500_000_000)  # 500.000000 ms
            Loss: 123,456 nanoseconds (acceptable for sensor data)
        """
        assert self.robot_id is not None
        assert self.topic_id is not None
        assert self.secs is not None
        assert self.msecs is not None
        time = rospy.Time(self.secs, self.msecs*1000000)
        return self.robot_id, self.topic_id, time

if __name__ == "__main__":
    import random
    import numpy as np
    import pdb
    import sys

    import argparse
    ap = argparse.ArgumentParser()
    # Add an argument to decide which test to run
    ap.add_argument("-t", "--test", required=True, help="Test to run")
    args = vars(ap.parse_args())

    if args["test"] == "hash":
        expected_collision = np.sqrt(2**Hash.HASH_LENGTH_BITS)
        MAX_DATA = 10000000000
        collision_i = np.array([])
        for loop in range(10):
            hashes = set()
            for i in range(MAX_DATA):
                randstr = str(random.random())
                hi = Hash(randstr.encode()).digest()
                if hi in hashes:
                    print(loop, "- Collision on hash %d -" % i, hi)
                    collision_i = np.append(collision_i, i)
                    break
                hashes.add(hi)
                if i % 1000000 == 0:
                    # print with only 3 decimals
                    print(f"{loop} {i}" +
                          f"Expected rate {(i/expected_collision):.3f}")
        print("Avg collision: %f" % np.average(collision_i))
    elif args["test"] == "time":
        for i in range(100):
            random_robot = random.randint(0, 255)
            random_topic = random.randint(0, 255)
            random_time = rospy.Time(random.randint(0, 65536),
                                     random.randint(0, 1000000000))
            ts = TsHeader.from_data(random_robot, random_topic, random_time)
            bindigest = ts.bindigest()
            ts2 = TsHeader.from_header(bindigest)
            robot_id, topic_id, time = ts2.get_id_and_time()
            assert robot_id == random_robot
            assert topic_id == random_topic
            assert time.secs == random_time.secs
            assert np.abs(time.nsecs-random_time.nsecs) < 1000000
    else:
        sys.exit("Invalid test")

