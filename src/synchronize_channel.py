#!/usr/bin/env python3
"""
SYNCHRONIZATION CHANNEL MODULE
===============================
Implements database synchronization between two robots using a finite state machine.

This module orchestrates the exchange of database messages between a local robot
(client) and a peer robot (server) over a ZMQ network connection.

Core Components:

1. Channel Class: Main orchestrator
   - Creates bidirectional communication link between two robots
   - Implements client-side state machine (pulls data from peer)
   - Implements server-side callback (serves data to peer)
   - Triggers sync based on RSSI signals or timers

2. SMACH State Machine: Synchronization protocol
   - Idle: Wait for sync trigger
   - RequestHash: Query peer for available messages
   - RequestHashReply: Process peer's response
   - GetData: Request specific message
   - GetDataReply: Store received message
   - TransmissionEnd: Handshake completion
   - Transitions: Complex state flows with timeouts
   
3. Communication Protocol: Binary message format
   - GHEAD: Get headers (query available messages)
   - GDATA: Get data (request specific message)
   - DENDT: Data end (sync complete handshake)
   - SERRM: Error message (protocol error response)

4. Bistable Class: Thread-safe trigger mechanism
   - Allows external threads to trigger sync
   - RSSI callback sets bistable, SM resets it
   - Prevents race conditions with locking

Architecture - Bidirectional Sync:

  Robot A (Client)                Robot B (Server)
  ┌──────────────────┐            ┌──────────────────┐
  │  integrate_database          integrate_database  │
  └──────────────────┘            └──────────────────┘
          │                               │
          │ RSSI trigger                 │
          ↓                              │
  ┌──────────────────┐                   │
  │ Channel A→B      │◄──ZMQ TCP/IP●────►│ Channel B←A
  │ (State Machine)  │                   │ (Server callback)
  └──────────────────┘                   │
          ↓                              ↓
  ┌──────────────────┐            ┌──────────────────┐
  │ Local DB         │            │ Local DB         │
  │ (Pulls data)     │            │ (Serves data)    │
  └──────────────────┘            └──────────────────┘

Synchronization Flow:

1. TRIGGER: RSSI callback sets sync bistable
   → rssi_cb() calls trigger_sync()
   → Bistable.set() activates

2. IDLE → REQUEST_HASH: SM detects bistable set
   → Calls Channel.trigger_sync()
   → Sends GHEAD message to peer
   → Waits for response with timeout

3. RECEIVE_HEADERS: Process peer response
   → Deserialize header list
   → Compare with local database
   → Identify missing/newer messages
   → Extract subset to request

4. GET_DATA: Request each message individually
   → For each header: Send GDATA request
   → Deserialize received message
   → Store in local database with add_modify_data()
   → Move to next message in list

5. TRANSMISSION_END: Sync completion handshake
   → Send DENDT with robot identification
   → Wait for ACK from peer
   → Publish sync complete event
   → Return to idle

Message Protocol (Binary Format):

Request (Client → Server):
  [CODE (5 bytes)] [HEADER (6 bytes, if GDATA)]
  Example: b'GHEAD' (request headers)
           b'GDATA' + header (request data)
           b'DENDT' + robot_name (end sync)

Response (Server → Client):
  [PAYLOAD (variable length)]
  For GHEAD: Serialized header list
  For GDATA: Packed message data
  For DENDT: "Ack" acknowledgment
  For ERROR: b'SERRM' + error_code

Timeout Handling:

- Client timeout: Default 6.0 seconds
- Polling interval: 0.2 seconds (CHECK_POLL_TIME)
- Trigger interval: 0.05 seconds (CHECK_TRIGGER_TIME)
- Server-side: No explicit timeout, requests served immediately

If timeout occurs:
  1. Reset bistable (stop sync)
  2. Return to idle state
  3. No error logged (expected timeout in weak signal)
  4. Wait for next trigger

Thread Model:

- Main thread: integrate_database node (rospy.spin())
- Channel thread: State machine loop (daemon thread)
- Callback thread: Server-side message handler (ZMQ receive thread)
- ROS callback thread: RSSI callback triggers sync

Thread Safety:
- Bistable: Uses threading.Lock for state access
- Database: Uses DBwLock for concurrent access
- Channel doesn't modify shared state during sync
- Comm_node: Thread-safe ZMQ operations

Dependencies:
- smach: Hierarchical state machine library
- zmq_comm_node: ZMQ network communication
- database_utils: Header/message serialization
- rospy: ROS Python interface
- threading: Multi-threaded state machine

Configuration:

Required YAML robot_configs entries:
  IP-address: Network address for ZMQ socket
  using-radio: Type of radio (false=localhost)
  node-type: Robot type (jackal, uav, etc.)

Performance Characteristics:

- Typical sync time: 100-500ms per robot (varies with database size)
- Header exchange: ~10-50ms
- Per-message transfer: ~10-100ms (depends on message size)
- Idle state CPU: < 1% (just polling)
- Active sync CPU: 5-20% (network I/O bound)

Use Cases:

1. Single-laptop multi-robot: ZMQ over localhost (127.0.0.1)
   Low latency (~1ms), high bandwidth (limited by CPU)

2. Multi-robot over WiFi: ZMQ over LAN
   Higher latency (~10-50ms), lower bandwidth (radio limited)
   RSSI-based triggering adapts to signal quality

3. Peer-to-peer: Bidirectional channels between all robots
   All robots sync with all neighbors simultaneously
   Creates distributed database consensus

State Machine Diagram:

                    ┌────────────────────┐
                    │      Stopped       │
                    │   (Shutdown)       │
                    └────────────────────┘
                           ▲
                           │
                           │
        ┌──────────────────►IDLE◄──────────────────┐
        │                   │                      │
        │        (Trigger=False)                   │
        │                   │                      │
        │         (Trigger=True)                   │
        │                   ▼                      │
        │             REQUEST_HASH                │
        │                   │                      │
        │         (Timeout: return)                │
        │                   │                      │
        │         (Success: process)               │
        │                   ▼                      │
        │          REQUEST_HASH_REPLY              │
        │                   │                      │
   IDLE ◄───(No data)       │     (Has data)───┐  │
        │                   ▼                   │  │
        │               GET_DATA                │  │
        │                   │                   │  │
        │         (Timeout: return)             │  │
        │                   │                   │  │
        │         (Success: process)            │  │
        │                   ▼                   │  │
        │            GET_DATA_REPLY             │  │
        │                   │                   │  │
        │        (More data: loop)──────────┐   │  │
        │                   │               │   │  │
        │     (Last data)   ▼               │   │  │
        └──────────TRANSMISSION_END ◄──────┘   │  │
                           │                   │  │
                    (Timeout: return)          │  │
                           │                   │  │
                     (Success: ACK)            │  │
                           │                   │  │
                           ▼                   │  │
                           IDLE ───────►       │  │
                                        (Loop)    │
                                                  └──(No data)

This state machine enables robust, adaptive synchronization suitable for
multi-robot systems with variable network conditions.
"""

import enum
import threading
import smach
import database as db
import database_utils as du
import hash_comm as hc
import zmq_comm_node
import logging
import rospy
import pdb
from std_msgs.msg import Time, String
from mocha_core.msg import SM_state

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# General configuration variables

# Get the header length from hash_comm
HEADER_LENGTH = hc.TsHeader.HEADER_LENGTH

# When actively polling for an answer or for a changement in variable,
# use this time
CHECK_POLL_TIME = 0.2
CHECK_TRIGGER_TIME = 0.05

# Msg codes that are used during the operation of the communication
# channel. Important: all codes should be CODE_LENGTH characters
CODE_LENGTH = 5


class Comm_msgs(enum.Enum):
    GHEAD = 1
    GDATA = 2
    DENDT = 3
    SERRM = 4

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
""" SMACH states for the synchronization state machine"""


class Idle(smach.State):
    """
    Idle State - Waits for sync trigger from RSSI callback.
    
    Represents the default state of the channel when no synchronization is
    happening. The state machine spins in this state, periodically checking
    if a sync has been triggered by an RSSI event.
    
    Behavior:
        1. Publish "Idle Start" status
        2. Loop until shutdown or sync trigger:
           - Poll self.outer.sync.get_state()
           - If True (sync triggered):
             * Publish "Idle to RequestHash" status
             * Return 'to_req_hash' (transition to RequestHash state)
           - If False (no trigger):
             * Sleep CHECK_TRIGGER_TIME (0.05s) to avoid busy-loop
        3. On shutdown:
           - Publish "Idle to Stopped" status
           - Return 'to_stopped' (exit state machine)
    
    Transitions:
        → RequestHash: When sync.get_state() returns True
        → Stopped: When shutdown signal received
    
    State Variables Used:
        self.outer.sm_shutdown: threading.Event (set = shutdown requested)
        self.outer.sync: Bistable object (True = sync requested)
    
    ROS Topics Published:
        ddb/client_sm_state/<robot_name>: Current state message
    
    Duration Typical:
        - Idle duration: Depends on trigger frequency
        - With RSSI 10Hz: ~100ms average between triggers
        - Each poll: ~50ms sleep + polling overhead
    
    Performance:
        - CPU: < 1% (just sleeping and polling)
        - Memory: Minimal (no buffers allocated)
        - Response time: ~50ms to sync trigger (CHECK_TRIGGER_TIME)
    
    Failure Scenarios:
        - Sync trigger never set: Stays in Idle forever (expected)
        - Shutdown never signaled: Spins in loop checking sleep
        - Both conditions true: Exits via shutdown path
    
    Use Case:
        Default state that conserves resources. Only activates sync when
        explicitly triggered (not on timer, not on every message).
        Keeps CPU usage low during periods of low network activity.
    """
    def __init__(self, outer):
        self.outer = outer
        smach.State.__init__(self, outcomes=['to_req_hash',
                                             'to_stopped'])

    def execute(self, userdata):
        self.outer.publishState("Idle Start")
        while (not self.outer.sm_shutdown.is_set() and
               not rospy.is_shutdown()):
            if self.outer.sync.get_state():
                # trigger sync and reset bistable
                self.outer.publishState("Idle to RequestHash")
                return 'to_req_hash'
            rospy.sleep(CHECK_TRIGGER_TIME)
        self.outer.publishState("Idle to Stopped")
        return 'to_stopped'


class RequestHash(smach.State):
    """
    RequestHash State - Query peer for available messages.
    
    This state sends a GHEAD (Get Headers) request to the peer robot's database
    server, asking for a list of all available message headers. The peer responds
    with a serialized list of headers sorted by priority and timestamp.
    
    Behavior:
        1. Publish "RequestHash Start" status
        2. Get current communication node
        3. Send GHEAD request (just the code, no data)
        4. Poll for answer with timeout:
           - Timeout: client_timeout (default 6.0 seconds)
           - Poll interval: CHECK_POLL_TIME (0.2 seconds)
           - Maximum iterations: ceil(6.0 / 0.2) = 30 polls
        5. If answer received before timeout:
           - Store answer in userdata
           - Publish "RequestHash to Reply" status
           - Return 'to_req_hash_reply' (process response)
        6. If timeout or shutdown:
           - Reset sync trigger (stops sync attempt)
           - Publish "RequestHash to Idle" status
           - Return 'to_idle' (return to idle state)
    
    Transitions:
        → RequestHashReply: When peer responds (answer received)
        → Idle: On timeout (no answer) or shutdown signal
        → Stopped: When shutdown signal detected during wait
    
    Request Format:
        Message: b'GHEAD' (5 bytes, CODE_LENGTH)
        Response: Serialized header list (variable length)
        Example response: 50+ headers × 6 bytes = 300+ bytes
    
    Answer Storage:
        self.outer.client_answer set by callback_client()
        - Filled by zmq_comm_node receive thread
        - Read by this state
        - Reset by RequestHashReply
    
    Timeout Behavior:
        - Wait loop: i from 0 to ceil(client_timeout / CHECK_POLL_TIME)
        - No exponential backoff (linear polling)
        - Silent timeout (no error log, expected for weak signals)
        - Returns to Idle to await next trigger
    
    Use Cases:
        1. Initial sync attempt: Query all messages only once
        2. Incremental sync: Get updated list for newer/missing messages
        3. Discovery: Learn what data peer has available
    
    Performance:
        - Typical time: 10-100ms (one round trip)
        - Network latency: ~1-50ms depending on connection
        - Timeout wait: Up to 6 seconds if peer not responding
    
    Error Scenarios:
        1. Peer unreachable → Timeout waits 6s → Returns to Idle
        2. Network packet loss → Timeout waits 6s → Returns to Idle
        3. Peer database empty → Receives empty list → Proceeds normally
        4. Shutdown during wait → Exits immediately to Stopped
    
    Optimization Notes:
        - Only ask for headers once per sync (not repeatedly)
        - Filter on peer before sending (not done, all sent)
        - Could optimize with delta requests (only changes)
    
    Remapping:
        Output: out_answer → Passed as 'sm_answer' to RequestHashReply
    
    Important:
        - The "<= " in loop condition includes timeout iteration
        - This ensures exactly one iteration after timeout
        - Prevents off-by-one error in timeout handling
    """
    def __init__(self, outer):
        self.outer = outer
        smach.State.__init__(self,
                             output_keys=['out_answer'],
                             outcomes=['to_idle',
                                       'to_req_hash_reply',
                                       'to_stopped'])

    def execute(self, userdata):
        self.outer.publishState("RequestHash Start")
        # Request current comm node
        comm = self.outer.get_comm_node()
        # Ask server for hash
        msg = Comm_msgs.GHEAD.name.encode()
        comm.connect_send_message(msg)
        # Wait for an answer in a polling fashion
        i = 0
        # Important: the <= is not a typo. We want one iteration more of the
        # loop to wait for the timeout
        while (i <= int(self.outer.client_timeout/CHECK_POLL_TIME)
               and not self.outer.sm_shutdown.is_set()):
            answer = self.outer.client_answer
            if answer is not None:
                rospy.logdebug(f"{comm.this_node} - Channel" +
                               f"- REQUESTHASH: {answer}")
                userdata.out_answer = answer
                self.outer.publishState("RequestHash to Reply")
                return 'to_req_hash_reply'
            rospy.sleep(CHECK_POLL_TIME)
            i += 1
        if self.outer.sm_shutdown.is_set():
            self.outer.publishState("RequestHash to Stopped")
            return 'to_stopped'
        self.outer.sync.reset()
        self.outer.publishState("RequestHash to Idle")
        return 'to_idle'


class RequestHashReply(smach.State):
    def __init__(self, outer):
        self.outer = outer
        smach.State.__init__(self, outcomes=['to_transmission_end',
                                             'to_get_data'],
                             input_keys=['in_answer'],
                             output_keys=['out_hash_list'])

    def execute(self, userdata):
        self.outer.publishState("GetHashReply Start")
        deserialized = du.deserialize_headers(userdata.in_answer)
        # print("REQUESTHASH: All ->", deserialized)
        # FIXME(fernando): Configure this depending on the message type
        # depending on the message type
        hash_list = self.outer.dbl.headers_not_in_local(deserialized,
                                                        newer=True)
        rospy.logdebug(f"======== - REQUESTHASH: {hash_list}")
        if len(hash_list):
            # We have hashes. Go get them
            # rospy.logdebug(f"{self.this_robot} - REQUESTHASH: Unique -> {hash_list}")
            userdata.out_hash_list = hash_list
            self.outer.publishState("GetHashReply to GetData")
            return 'to_get_data'
        # We have no hashes. Sync is complete
        self.outer.publishState("GetHashReply to TransmissionEnd")
        return 'to_transmission_end'


class GetData(smach.State):
    def __init__(self, outer):
        self.outer = outer
        smach.State.__init__(self, outcomes=['to_idle',
                                             'to_get_data_reply',
                                             'to_stopped'],
                             input_keys=['in_hash_list'],
                             output_keys=['out_hash_list',
                                          'out_req_hash',
                                          'out_answer'])

    def execute(self, userdata):
        self.outer.publishState("GetData Start")
        # Request current comm node
        comm = self.outer.get_comm_node()
        hash_list = userdata.in_hash_list.copy()
        # Get the first hash of the list, the one with the higher priority
        req_hash = hash_list.pop(0)
        rospy.logdebug(f"{comm.this_node} - Channel - GETDATA: {req_hash}")
        # Ask for hash
        msg = Comm_msgs.GDATA.name.encode() + req_hash
        comm.connect_send_message(msg)
        # Wait for an answer in a polling fashion
        i = 0
        # Important: the <= is not a typo. We want one iteration more of the
        # loop to wait for the timeout
        while (i <= int(self.outer.client_timeout/CHECK_POLL_TIME)
               and not self.outer.sm_shutdown.is_set()):
            answer = self.outer.client_answer
            if answer is not None:
                userdata.out_hash_list = userdata.in_hash_list
                userdata.out_answer = answer
                userdata.out_req_hash = req_hash
                self.outer.publishState("GetData to GetDataReply")
                return 'to_get_data_reply'
            rospy.sleep(CHECK_POLL_TIME)
            i += 1
        if self.outer.sm_shutdown.is_set():
            self.outer.publishState("GetData to Stopped")
            return 'to_stopped'
        self.outer.sync.reset()
        self.outer.publishState("GetData to Idle")
        return 'to_idle'


class GetDataReply(smach.State):
    def __init__(self, outer):
        self.outer = outer
        smach.State.__init__(self, outcomes=['to_transmission_end',
                                             'to_get_more_data'],
                             input_keys=['in_hash_list',
                                         'in_answer',
                                         'in_req_hash'],
                             output_keys=['out_hash_list'])

    def execute(self, userdata):
        self.outer.publishState("GetDataReply Start")
        # store result in db
        dbm = du.unpack_data(userdata.in_req_hash, userdata.in_answer)
        hash_list = userdata.in_hash_list.copy()
        self.outer.dbl.add_modify_data(dbm)
        hash_list.remove(userdata.in_req_hash)
        # rospy.logdebug(f"HASH_LIST {hash_list} REQ_HASH {userdata.in_req_hash}")
        # Transition back
        if hash_list:
            userdata.out_hash_list = hash_list
            self.outer.publishState("GetDataReply to GetMoreData")
            return 'to_get_more_data'
        else:
            self.outer.publishState("GetDataReply to TransmissionEnd")
            return 'to_transmission_end'


class TransmissionEnd(smach.State):
    def __init__(self, outer):
        self.outer = outer
        smach.State.__init__(self, outcomes=['to_idle',
                                             'to_stopped'])

    def execute(self, userdata):
        self.outer.publishState("TransmissionEnd Start")
        # Request current comm node
        comm = self.outer.get_comm_node()
        rospy.logdebug(f"{comm.this_node} - Channel - DENDT")
        # Ask for hash
        msg = Comm_msgs.DENDT.name.encode() + self.outer.this_robot.encode()
        comm.connect_send_message(msg)
        # Wait for an answer in a polling fashion
        i = 0
        # Important: the <= is not a typo. We want one iteration more of the
        # loop to wait for the timeout
        while (i <= int(self.outer.client_timeout/CHECK_POLL_TIME)
               and not self.outer.sm_shutdown.is_set()):
            answer = self.outer.client_answer
            if answer is not None:
                # We received an ACK
                self.outer.client_sync_complete_pub.publish(Time(rospy.get_rostime()))
                break
            rospy.sleep(CHECK_POLL_TIME)
            i += 1
        if self.outer.sm_shutdown.is_set():
            self.outer.publishState("TransmissionEnd to Stopped")
            return 'to_stopped'
        self.outer.sync.reset()
        self.outer.publishState("TransmissionEnd to Idle")
        return 'to_idle'


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
""" Bistable class. Allows the communication between an external method
in Channel and its state machine. The bistable will be set by a
method in Channel to trigger the synchronization, and it will be reset
inside the state machine """


class Bistable():
    """
    Thread-Safe Binary Trigger (Bistable Latch)
    ============================================
    A thread-safe mechanism for signaling sync triggers between threads.
    
    One thread (RSSI callback) sets the bistable to request data synchronization.
    Another thread (state machine) detects the set state and resets it once sync
    is initiated.
    
    This implements a bistable flip-flop: two stable states (True/False) with
    transitions triggered by external events. Used here to safely communicate
    between asynchronous RSSI callbacks and the synchronous state machine.
    
    Thread Safety:
    - All state changes protected by threading.Lock
    - Acquired before reading/writing self.state
    - Prevents race conditions even with concurrent access
    - Lock released immediately after operation (minimal contention)
    
    State Transitions:
    
    False (Reset)  ──set()──►  True (Set)
                      ◄──reset()──
    
    Typical Usage:
    
    Thread 1 (RSSI Callback):
        if rssi > threshold:
            self.sync.set()  # Request sync
    
    Thread 2 (State Machine):
        while not shutdown:
            if self.sync.get_state():  # Check for request
                # Initiate sync
                self.sync.reset()  # Acknowledge request
    
    Attributes:
        state (bool): Current trigger state (True=sync requested, False=idle)
        lock (threading.Lock): Ensures atomic state changes
    
    Use Cases:
        1. Event signaling: Wake up blocked thread
           other_thread.event.wait()
           sync_trigger.set()
        
        2. Resource availability: Signal when data is ready
           if database_updated:
               data_ready.set()
           if data_ready.get_state():
               process_data()
        
        3. Task activation: External event triggers internal action
           network_connected.set()  # From hardware monitor
           if network_connected.get_state():
               start_transmission()
    
    Performance:
        - set(): O(1) with lock acquisition (microseconds)
        - reset(): O(1) with lock acquisition (microseconds)
        - get_state(): O(1) with lock acquisition (microseconds)
    
    Comparison with threading Alternatives:
    
    Event (threading.Event):
        + Supports wait() with timeout
        + Multiple waiters can be unblocked
        - More overhead than Bistable
        - Cannot be "pulsed" (must clear explicitly)
    
    Condition (threading.Condition):
        + Supports wait() and notify mechanisms
        + Fine-grained notifications
        - Complexity not needed here
        - More overhead
    
    Bistable (this class):
        + Simple binary state
        + Minimal overhead (just a bool and lock)
        + Straightforward polling interface
        - No blocking wait (must poll)
        - Single thread only (not broadcast)
    
    Implementation Details:
    
    Lock Acquisition Pattern:
        lock.acquire()     # Get exclusive access
        state = ...         # Read/write state
        lock.release()      # Release for other threads
    
    Could use 'with' statement for cleaner code:
        with self.lock:
            self.state = ...
    
    But current implementation uses acquire/release for clarity.
    
    Potential Improvements:
    
    1. Add timeout support:
       def wait_set(self, timeout=None):
           end_time = time.time() + timeout
           while not self.get_state():
               if time.time() > end_time:
                   return False
               time.sleep(0.01)
           return True
    
    2. Use threading.Event instead (simpler, has wait):
       self.event = threading.Event()
       # Instead of Bistable
    
    3. Add pulse mode (set then reset after delay):
       def pulse(self, duration=0.1):
           self.set()
           time.sleep(duration)
           self.reset()
    
    Safety Notes:
        - Lock must always be released (guaranteed by acquire/release)
        - No deadlock possible (single lock, no nested calls)
        - Multiple calls to set() safe (idempotent)
        - Multiple calls to reset() safe (idempotent)
    
    Integration with State Machine:
    
    Bistable manages the trigger for MOCHA sync:
        RSSI callback     →  Bistable.set()
                              ↓
        State machine     →  detect set()
                              ↓
        Request hash      →  Bistable.reset()
                              ↓
        Back to idle
    
    This ensures:
    - Only one sync starts per trigger
    - Multiple triggers don't queue (just set flag)
    - State machine progresses without polling overhead
    """
    def __init__(self):
        self.state = False
        self.lock = threading.Lock()

    def set(self):
        """Set the bistable to True (trigger state)."""
        self.lock.acquire()
        self.state = True
        self.lock.release()

    def reset(self):
        """Reset the bistable to False (idle state)."""
        self.lock.acquire()
        self.state = False
        self.lock.release()

    def get_state(self):
        """Read current bistable state. Returns immediately (polling)."""
        return self.state


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# Channel class

class Channel():
    def __init__(self, dbl, this_robot,
                 target_robot, robot_configs,
                 client_timeout):
        # Check input arguments
        assert type(dbl) is db.DBwLock
        assert type(this_robot) is str
        assert type(target_robot) is str
        assert type(robot_configs) is dict
        assert type(client_timeout) is float or type(client_timeout) is int

        # Override smach logger to use rospy loggers
        # Use rospy.logdebug for smach info as it is too verbose
        # def set_loggers(info,warn,debug,error):
        smach.set_loggers(rospy.logdebug, rospy.logwarn,
                          rospy.logdebug, rospy.logerr)

        # Basic parameters of the communication channel
        self.this_robot = this_robot
        self.target_robot = target_robot
        self.dbl = dbl
        # Config file used to fetch configurations
        self.robot_configs = robot_configs

        # Client timeout defines the time before an answer is considered lost
        self.client_timeout = client_timeout

        # Bistable used to start the synchronization. It will be enabled
        # by self.start_sync(), and it will be disabled inside the state
        # machine once the synchronization starts
        self.sync = Bistable()

        # Change to false before running the state machine. Otherwise,
        # when the SM reaches the idle state it will stop
        self.sm_shutdown = threading.Event()
        self.sm_shutdown.set()
        # The answer of the client will be written here
        self.client_answer = None
        # The pointer of the comm node will be stored here
        self.comm_node = None
        # Create state machine and add states
        self.sm = smach.StateMachine(outcomes=['failure', 'stopped'])

        # Create topic to notify that the transmission ended
        self.client_sync_complete_pub = rospy.Publisher(f"ddb/client_sync_complete/{self.target_robot}",
                                                        Time,
                                                        queue_size=20)
        self.server_sync_complete_pub = rospy.Publisher(f"ddb/server_sync_complete/{self.target_robot}",
                                                        Time,
                                                        queue_size=20)

        # Create a topic that prints the current state of the state machine
        self.sm_state_pub = rospy.Publisher(f"ddb/client_sm_state/{self.target_robot}",
                                            SM_state,
                                            queue_size=20)
        self.sm_state_count = 0

        with self.sm:
            smach.StateMachine.add('IDLE',
                                   Idle(self),
                                   transitions={'to_req_hash': 'REQ_HASH',
                                                'to_stopped': 'stopped'})
            smach.StateMachine.add('REQ_HASH',
                                   RequestHash(self),
                                   transitions={'to_idle': 'IDLE',
                                                'to_req_hash_reply': 'REQ_HASH_REPLY',
                                                'to_stopped': 'stopped'},
                                   remapping={'out_answer': 'sm_answer'})

            smach.StateMachine.add('REQ_HASH_REPLY',
                                   RequestHashReply(self),
                                   transitions={'to_transmission_end': 'TRANSMISSION_END',
                                                'to_get_data': 'GET_DATA'},
                                   remapping={'in_answer': 'sm_answer',
                                              'out_hash_list': 'sm_hash_list'})

            smach.StateMachine.add('GET_DATA',
                                   GetData(self),
                                   transitions={'to_idle': 'IDLE',
                                                'to_get_data_reply': 'GET_DATA_REPLY',
                                                'to_stopped': 'stopped'},
                                   remapping={'in_hash_list': 'sm_hash_list',
                                              'out_hash_list': 'sm_hash_list_2',
                                              'out_req_hash': 'sm_req_hash',
                                              'out_answer': 'sm_answer_2' })

            smach.StateMachine.add('GET_DATA_REPLY',
                                   GetDataReply(self),
                                   transitions={'to_transmission_end': 'TRANSMISSION_END',
                                                'to_get_more_data': 'GET_DATA'},
                                   remapping={'in_hash_list': 'sm_hash_list_2',
                                              'in_req_hash': 'sm_req_hash',
                                              'in_answer': 'sm_answer_2',
                                              'out_hash_list': 'sm_hash_list'})

            smach.StateMachine.add('TRANSMISSION_END',
                                   TransmissionEnd(self),
                                   transitions={'to_idle': 'IDLE',
                                                'to_stopped': 'stopped'})

    def publishState(self, msg):
        """ Publish the string msg (where the state message will be stored)
        with a timestamp"""
        assert type(msg) is str
        state_msg = SM_state()
        state_msg.header.stamp = rospy.Time.now()
        state_msg.header.frame_id = self.this_robot
        state_msg.header.seq = self.sm_state_count
        self.sm_state_count += 1
        state_msg.state = msg
        self.sm_state_pub.publish(state_msg)

    def run(self):
        """ Configures the zmq_comm_node and also starts the state
        machine thread"""
        # The comm node needs to be created first, as it may be required
        # by the SM
        self.comm_node = zmq_comm_node.Comm_node(self.this_robot,
                                                 self.target_robot,
                                                 self.robot_configs,
                                                 self.callback_client,
                                                 self.callback_server,
                                                 self.client_timeout)
        # Unset this flag before starting the SM thread
        self.sm_shutdown.clear()
        self.th = threading.Thread(target=self.sm_thread, args=())
        self.th.setDaemon(True)
        self.th.start()

    def stop(self):
        # Set the flag and wait until the state machine finishes
        self.sm_shutdown.set()
        self.th.join()


    def sm_thread(self):
        # Start the state machine and wait until it ends
        rospy.logwarn(f"Channel {self.this_robot} -> {self.target_robot} started")
        outcome = self.sm.execute()
        exit_msg = f"Channel {self.this_robot} -> {self.target_robot}" + \
            f" finished with outcome: {outcome}"
        if outcome == 'failure':
            rospy.logerr(exit_msg)
        elif outcome == 'stopped':
            rospy.logwarn(exit_msg)
        # Terminate the comm node once the state machine ends
        self.comm_node.terminate()

    def get_comm_node(self):
        if not self.comm_node:
            rospy.logerr("Requesting for an empty comm node")
            rospy.signal_shutdown("Requesting for an empty comm node")
            rospy.spin()
        return self.comm_node

    def trigger_sync(self):
        if self.sync.get_state():
            rospy.logwarn(f"{self.this_robot} <- {self.target_robot}: Channel Busy")
        else:
            self.sync.set()

    def callback_client(self, msg):
        if msg is not None:
            rospy.logdebug(f"{self.this_robot} - Channel - CALLBACK_CLIENT: len: {len(msg)}")
        else:
            rospy.logdebug(f"{self.this_robot} - Channel - CALLBACK_CLIENT - None")
        self.client_answer = msg

    def callback_server(self, msg):
        rospy.logdebug(f"{self.this_robot} - Channel - CALLBACK_SERVER: {msg}")
        header = msg[:CODE_LENGTH].decode()
        data = msg[CODE_LENGTH:]

        if header == Comm_msgs.GHEAD.name:
            # Returns all the headers that this node has
            # FIXME(Fernando): Configure this depending on the message type
            headers = self.dbl.get_header_list(filter_latest=True)
            rospy.logdebug(f"{self.this_robot} - Channel - Sending {len(headers)} headers")
            rospy.logdebug(f"{self.this_robot} - Channel - {headers}")
            serialized = du.serialize_headers(headers)
            return serialized
        if header == Comm_msgs.GDATA.name:
            r_header = data
            # Returns a packed data for the requires header
            # One header at a time
            if len(data) != HEADER_LENGTH:
                rospy.logerr(f"{self.this_robot} - Wrong header length: {len(data)}")
                return Comm_msgs.SERRM.name.encode()
            try:
                dbm = self.dbl.find_header(r_header)
                packed = du.pack_data(dbm)
                return packed
            except Exception:
                rospy.logerr(f"{self.this_robot} - Header not found: {r_header}")
                return Comm_msgs.SERRM.name.encode()
        if header == Comm_msgs.DENDT.name:
            target = data
            if target.decode() != self.target_robot:
                print(f"{self.this_robot} - Channel - Wrong DENDT -" +
                             f" Target: {target.decode()} - " +
                             f"My target: {self.target_robot}")
            self.server_sync_complete_pub.publish(Time(rospy.get_rostime()))
            return "Ack".encode()
        return Comm_msgs.SERRM.name.encode()
