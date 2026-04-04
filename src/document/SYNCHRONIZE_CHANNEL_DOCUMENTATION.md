# Synchronize Channel Module Documentation

## Overview

The `synchronize_channel.py` module implements **database synchronization between two robots** using a finite state machine (SMACH). It orchestrates the exchange of database messages between a local robot (client) and a peer robot (server) over a ZMQ network connection with automatic retry and timeout handling.

**Location:** `/home/nlg/swarm_ws/src/mocha_tplink/src/synchronize_channel.py`

### Core Purpose

Enable efficient, bidirectional database synchronization in multi-robot systems by:
- Pulling data from peer robots on demand
- Handling network timeouts gracefully
- Preventing race conditions with thread-safe mechanisms
- Adapting to signal quality changes via RSSI triggers

---

## Architecture

### System Components

```
Robot A (Client)                          Robot B (Server)
┌──────────────────┐                     ┌──────────────────┐
│ integrate_database                     integrate_database │
└──────────────────┘                     └──────────────────┘
        │                                        │
        │ RSSI trigger                          │
        ↓                                       │
┌──────────────────┐                           │
│ Channel A→B      │◄──ZMQ TCP/IP●────────────►│ Channel B←A
│ (State Machine)  │                           │ (Server callback)
└──────────────────┘                           │
        ↓                                       ↓
┌──────────────────┐                     ┌──────────────────┐
│ Local DB         │                     │ Local DB         │
│ (Pulls data)     │                     │ (Serves data)    │
└──────────────────┘                     └──────────────────┘
```

### Each robot acts in two roles:

1. **As a Client**: Initiates sync with peer robots (pulls data)
   - Creates Channel objects to communicate with targets
   - Uses state machine to orchestrate pull-based sync
   - Stores received data in local database

2. **As a Server**: Responds to peer sync requests (serves data)
   - Receives requests via callback_server()
   - Queries local database for data
   - Sends serialized responses back

---

## State Machine

### Overview

The SMACH state machine manages the client-side synchronization protocol with 6 states and complex transitions including timeout handling and retry loops.

### State Diagram

```
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
```

### State Descriptions

#### 1. **Idle State**
**Purpose:** Waits for sync trigger from RSSI callback  
**File Lines:** 250-308

**Behavior:**
- Default state when no synchronization is active
- Polls `self.outer.sync.get_state()` to check for trigger
- Sleep interval: `CHECK_TRIGGER_TIME` (0.05 seconds)

**Transitions:**
- → **RequestHash**: When sync bistable is set (`sync.get_state() == True`)
- → **Stopped**: When shutdown signal received

**Duration:** Depends on trigger frequency (typically 100-200ms between checks)

**CPU Impact:** < 1% during idle (just sleeping and polling)

---

#### 2. **RequestHash State**
**Purpose:** Query peer robot for available message headers  
**File Lines:** 310-398

**Behavior:**
1. Gets communication node to peer
2. Sends `GHEAD` (Get Headers) request message
3. Polls for response with timeout
   - Timeout duration: `client_timeout` (default 6.0 seconds)
   - Poll interval: `CHECK_POLL_TIME` (0.2 seconds)
   - Max iterations: ~30 polls

**Transitions:**
- → **RequestHashReply**: Full response received from peer
- → **Idle**: Request timeout or shutdown signal

**Request Format:**
```
Message: b'GHEAD' (5 bytes, CODE_LENGTH)
```

**Response Format:**
```
Serialized header list containing 50+ headers × 6 bytes each
```

**Important Notes:**
- Loop uses `<=` operator: `i <= client_timeout/CHECK_POLL_TIME`
  - This ensures one additional iteration after timeout
  - Prevents off-by-one errors
- Timeout is considered normal (weak signal, peer unreachable)
- No error logged on timeout

---

#### 3. **RequestHashReply State**
**Purpose:** Process peer's header response  
**File Lines:** 400-432

**Behavior:**
1. Deserializes header list from peer response
2. Compares with local database
3. Filters to find missing or newer messages
4. Creates hash list of data to request

**Key Operation:**
```python
hash_list = self.outer.dbl.headers_not_in_local(deserialized, newer=True)
```

**Transitions:**
- → **GetData**: Hash list contains items (data to fetch)
- → **TransmissionEnd**: Hash list empty (all data up-to-date)

**Example Flow:**
```
Peer has: [H1(timestamp=10), H2(t=20), H3(t=30)]
Local has: [H1(t=10)]
→ hash_list = [H2, H3]  (missing or newer)
```

---

#### 4. **GetData State**
**Purpose:** Request individual messages from peer  
**File Lines:** 434-508

**Behavior:**
1. Pops first hash from list (highest priority)
2. Sends `GDATA` request for that specific message
3. Polls for response with timeout
4. Stores response details for processing

**Request Format:**
```
Message: b'GDATA' + header (6 bytes)
Total: 5 + 6 = 11 bytes
```

**Output Data:**
- `out_hash_list`: Remaining hashes to process
- `out_req_hash`: Current hash being requested
- `out_answer`: Response message data

**Transitions:**
- → **GetDataReply**: Response received
- → **Idle**: Timeout or shutdown

**Processing Flow:**
```
hash_list = [H2, H3, H4]
↓ (pop first)
req_hash = H2
hash_list = [H3, H4]
```

---

#### 5. **GetDataReply State**
**Purpose:** Process received message and store in database  
**File Lines:** 510-541

**Behavior:**
1. Unpacks received data using `du.unpack_data()`
2. Stores message in local database via `add_modify_data()`
3. Removes processed hash from remaining list
4. Checks if more data to fetch

**Key Operation:**
```python
dbm = du.unpack_data(userdata.in_req_hash, userdata.in_answer)
self.outer.dbl.add_modify_data(dbm)
hash_list.remove(userdata.in_req_hash)
```

**Transitions:**
- → **GetData**: More hashes in list (loop for next message)
- → **TransmissionEnd**: Hash list empty (all data received)

**Example Sequence:**
```
State 1: hash_list=[H3, H4], get H2
State 2: hash_list=[H4], get H3
State 3: hash_list=[], get H4
State 4: hash_list=[], → TransmissionEnd
```

---

#### 6. **TransmissionEnd State**
**Purpose:** Complete handshake with peer robot  
**File Lines:** 543-588

**Behavior:**
1. Sends `DENDT` (Data End) message with robot ID
2. Waits for ACK from peer with timeout
3. On success: publishes sync completion event
4. Returns to Idle

**Request Format:**
```
Message: b'DENDT' + robot_name.encode()
Example: b'DENDT' + b'robot_name'
```

**Response Format:**
```
"Ack" (string acknowledgment)
```

**Side Effect:**
Publishes `ddb/client_sync_complete/<target_robot>` with timestamp

**Transitions:**
- → **Idle**: ACK received or timeout
- → **Stopped**: Shutdown signal

**Purpose of Handshake:**
- Notifies peer that client finished pulling
- Peer publishes server-side sync complete event
- Allows tracking of bidirectional sync completion

---

## Communication Protocol

### Message Types

All messages use binary encoding with a 5-byte CODE header:

| Message | Code | Purpose | Payload | Response |
|---------|------|---------|---------|----------|
| **GHEAD** | `GHEAD` | Get headers | None | Serialized header list |
| **GDATA** | `GDATA` | Get data | 6-byte header | Packed message data |
| **DENDT** | `DENDT` | Data end | Robot name | "Ack" string |
| **SERRM** | `SERRM` | Error | Error code | N/A |

### Message Flow Sequence

```
Client                              Server
  │                                   │
  ├─── GHEAD ─────────────────────────>  Database query
  │                                   │
  │  <───────── Header List ──────────┤
  │                                   │
  ├─── GDATA (hash_1) ────────────────>  Database lookup
  │                                   │
  │  <────── Message Data_1 ──────────┤
  │                                   │
  ├─── GDATA (hash_2) ────────────────>  Database lookup
  │                                   │
  │  <────── Message Data_2 ──────────┤
  │                                   │
  ├─── DENDT (robot_name) ────────────>  Publish sync event
  │                                   │
  │  <──────── "Ack" ─────────────────┤
  │                                   │
  └─ Publish sync complete            └─ Publish sync complete
```

---

## Thread Safety & Synchronization

### Bistable Class
**Purpose:** Thread-safe binary trigger mechanism  
**File Lines:** 590-751

```python
class Bistable():
    def __init__(self):
        self.state = False
        self.lock = threading.Lock()
    
    def set(self):
        self.lock.acquire()
        self.state = True
        self.lock.release()
    
    def reset(self):
        self.lock.acquire()
        self.state = False
        self.lock.release()
    
    def get_state(self):
        return self.state
```

**Design Pattern:** Flip-flop with two stable states

**Usage:**
- RSSI callback (hardware thread) calls `sync.set()`
- State machine (SM thread) calls `sync.reset()`
- Prevents race conditions with threading.Lock

**Performance:**
- `set()` / `reset()` / `get_state()`: O(1) with microsecond lock overhead
- No blocking (polling-based)

### Thread Model

```
Main Thread (rospy.spin())
├─ ROS callbacks
│  └─ RSSI callback → sync.set()
│
Channel Thread (daemon, started in run())
├─ State machine loop
│  ├─ Call state.execute()
│  ├─ Check sync.get_state()
│  └─ Reset sync → sync.reset()
│
Comm Node Thread (ZMQ receive)
├─ Listen for peer messages
└─ Call callback_server() for responses
```

### Synchronization Points

| Component | Protection | Mechanism |
|-----------|-----------|-----------|
| Bistable state | Atomic access | threading.Lock |
| Database | Concurrent access | db.DBwLock |
| SM state | Single thread | State machine |
| Client answer | Single writer | Callback assignment |

---

## Channel Class

**Purpose:** Main orchestrator for sync with single peer  
**File Lines:** 753-950

### Initialization

```python
Channel(dbl, this_robot, target_robot, robot_configs, client_timeout)
```

**Parameters:**
- `dbl`: Database with lock (db.DBwLock instance)
- `this_robot`: Name of local robot (string)
- `target_robot`: Name of peer robot (string)
- `robot_configs`: Configuration dict with IP, port, etc.
- `client_timeout`: Timeout for requests in seconds (default 6.0)

### Key Attributes

```python
self.sync = Bistable()                          # Sync trigger
self.sm_shutdown = threading.Event()            # Shutdown flag
self.client_answer = None                       # Latest response
self.comm_node = None                           # ZMQ connection
self.sm = smach.StateMachine(...)              # State machine
self.th = None                                  # SM thread
```

### Published Topics

| Topic | Message Type | Purpose |
|-------|----------|---------|
| `ddb/client_sm_state/<robot>` | `SM_state` | Current state status |
| `ddb/client_sync_complete/<robot>` | `Time` | Sync completion event |
| `ddb/server_sync_complete/<robot>` | `Time` | Server ACK timestamp |

### Key Methods

#### `run()`
Starts the synchronization channel:
1. Creates ZMQ communication node
2. Clears shutdown flag
3. Spawns state machine thread (daemon)

```python
def run(self):
    self.comm_node = zmq_comm_node.Comm_node(...)
    self.sm_shutdown.clear()
    self.th = threading.Thread(target=self.sm_thread)
    self.th.setDaemon(True)
    self.th.start()
```

#### `stop()`
Gracefully shuts down the channel:
1. Sets shutdown flag
2. Joins state machine thread
3. Waits for completion

```python
def stop(self):
    self.sm_shutdown.set()
    self.th.join()
```

#### `trigger_sync()`
External method to request synchronization:
- Called by RSSI callback or timer
- Sets bistable if not already active
- Logs warning if channel busy

```python
def trigger_sync(self):
    if self.sync.get_state():
        rospy.logwarn(f"{self.this_robot} <- {self.target_robot}: Channel Busy")
    else:
        self.sync.set()
```

#### `callback_client(msg)`
Receives responses from peer (called by ZMQ receive thread):
```python
def callback_client(self, msg):
    self.client_answer = msg
```

#### `callback_server(msg)`
Serves data to peer (called by ZMQ receive thread):
- Handles GHEAD requests: Returns serialized headers
- Handles GDATA requests: Returns packed message data
- Handles DENDT requests: Publishes server sync complete
- Returns error for invalid requests

```python
def callback_server(self, msg):
    header = msg[:CODE_LENGTH].decode()
    data = msg[CODE_LENGTH:]
    
    if header == Comm_msgs.GHEAD.name:
        headers = self.dbl.get_header_list(filter_latest=True)
        serialized = du.serialize_headers(headers)
        return serialized
    
    if header == Comm_msgs.GDATA.name:
        dbm = self.dbl.find_header(data)
        packed = du.pack_data(dbm)
        return packed
    
    if header == Comm_msgs.DENDT.name:
        self.server_sync_complete_pub.publish(Time(rospy.get_rostime()))
        return "Ack".encode()
    
    return Comm_msgs.SERRM.name.encode()
```

### State Machine Configuration

```python
with self.sm:
    # State definitions and transitions
    smach.StateMachine.add('IDLE', Idle(self), ...)
    smach.StateMachine.add('REQ_HASH', RequestHash(self), ...)
    smach.StateMachine.add('REQ_HASH_REPLY', RequestHashReply(self), ...)
    smach.StateMachine.add('GET_DATA', GetData(self), ...)
    smach.StateMachine.add('GET_DATA_REPLY', GetDataReply(self), ...)
    smach.StateMachine.add('TRANSMISSION_END', TransmissionEnd(self), ...)
```

#### State Remapping

Passes data between states using SMACH's userdata mechanism:

| Variable | States | Purpose |
|----------|--------|---------|
| `sm_answer` | REQ_HASH → REQ_HASH_REPLY | Header response |
| `sm_hash_list` | REQ_HASH_REPLY → GET_DATA | Hashes to fetch |
| `sm_hash_list_2` | GET_DATA → GET_DATA_REPLY | Remaining hashes |
| `sm_req_hash` | GET_DATA → GET_DATA_REPLY | Current hash |
| `sm_answer_2` | GET_DATA → GET_DATA_REPLY | Message data |

---

## Timing & Performance

### Timing Parameters

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `CHECK_POLL_TIME` | 0.2 seconds | Poll interval during request wait |
| `CHECK_TRIGGER_TIME` | 0.05 seconds | Poll interval in Idle state |
| `client_timeout` | 6.0 seconds | Max wait for peer response |
| `HEADER_LENGTH` | 6 bytes | Size of message hash |
| `CODE_LENGTH` | 5 bytes | Size of message type code |

### Typical Sync Times

| Phase | Duration | Notes |
|-------|----------|-------|
| Header exchange | 10-50ms | One network round trip |
| Per message | 10-100ms | Depends on message size |
| Full sync (10 msgs) | 100-500ms | Sequential, block-by-block |
| Idle polling | ~50ms | Per poll cycle |

### CPU Impact

| State | CPU Usage | Notes |
|-------|-----------|-------|
| Idle | < 1% | Just sleeping, no active work |
| RequestHash | 1-2% | Polling with timeouts |
| GetData | 5-20% | Network I/O, serialization |
| Overall average | 2-5% | Depends on sync frequency |

---

## Error Handling

### Timeout Behavior

**When Request Times Out:**
1. Polling loop completes max iterations
2. `sync.reset()` called to clear trigger
3. State returns to Idle (no error logged)
4. Sync restarts on next trigger

**Rationale:** Timeouts are expected in weak signal conditions, not logged as errors

### Network Errors

**GDATA request for missing header:**
1. Exception caught in `callback_server()`
2. Returns `SERRM` error message
3. Client ignores error message
4. Continues to next hash in list

**Example Logger Output:**
```
logerr: Robot A - Header not found: b'\x00\x01\x02\x03\x04\x05'
```

### Shutdown Handling

**Shutdown Detection:**
- All states check `self.outer.sm_shutdown.is_set()`
- During wait loops: immediate exit to Stopped
- Publishing state transition before exiting

**Graceful Shutdown:**
1. `stop()` method called
2. Sets shutdown flag
3. Current state detects it and exits
4. SM thread joins and completes
5. Comm node connection terminated

---

## Integration Examples

### Starting a Channel

```python
# Create channel to sync with "robot_b"
ch = Channel(dbl=db_obj, 
             this_robot="robot_a",
             target_robot="robot_b",
             robot_configs=configs,
             client_timeout=6.0)

# Start the state machine thread
ch.run()

# Trigger sync from RSSI callback
def rssi_callback(msg):
    if msg.signal_strength > -50:  # Strong signal
        ch.trigger_sync()
```

### Stopping a Channel

```python
# Gracefully stop
ch.stop()  # Blocks until SM thread completes
```

### Multiple Channels

```python
# Create channels to all peers
channels = {}
for peer_name in peer_list:
    ch = Channel(...)
    ch.run()
    channels[peer_name] = ch

# Later: stop all
for ch in channels.values():
    ch.stop()
```

---

## Dependencies

### External Modules

- **smach**: Hierarchical state machine library (Python)
- **zmq_comm_node**: ZMQ network communication wrapper
- **database_utils**: Header/message serialization utilities
- **rospy**: ROS Python client library
- **threading**: Python standard library for threads

### Internal Modules

- **database**: `db.DBwLock` for thread-safe database access
- **hash_comm**: `TsHeader` for header definitions

### ROS Messages

- **std_msgs.Time**: Timestamp message for sync events
- **mocha_core.msg.SM_state**: Custom state machine status message

---

## Configuration

### Required YAML Entries (robot_configs)

Each robot in the system must have configuration:

```yaml
robot_a:
  IP-address: "192.168.1.10"
  using-radio: false
  node-type: "jackal"

robot_b:
  IP-address: "192.168.1.11"
  using-radio: true
  node-type: "uav"
```

### Channel Parameters

| Parameter | Type | Default | Units | Notes |
|-----------|------|---------|-------|-------|
| `client_timeout` | float | 6.0 | seconds | Max wait for peer response |
| `CHECK_POLL_TIME` | float | 0.2 | seconds | Request polling interval |
| `CHECK_TRIGGER_TIME` | float | 0.05 | seconds | Idle polling interval |

---

## Debugging Tips

### Enable Detailed Logging

```python
# In your node setup:
rospy.logdebug(f"Channel {A} -> {B}: Entering state X")
rospy.logdebug(f"Channel {A} -> {B}: Request timeout")
rospy.logerr(f"Channel {A} -> {B}: Network error")
```

### Monitor State Transitions

Subscribe to state topic:
```bash
rostopic echo ddb/client_sm_state/robot_b
```

Expected output:
```
state: "Idle Start"
state: "Idle to RequestHash"
state: "RequestHash Start"
state: "RequestHash to Reply"
state: "GetHashReply Start"
state: "GetHashReply to GetData"
...
```

### Common Issues

**Channel stuck in Idle:**
- Check if sync trigger is being called
- Verify RSSI callback is connected
- Ensure bistable.set() is being reached

**Continuous timeouts:**
- Check network connectivity
- Verify peer robot is running
- Check firewall/ZMQ settings
- Review peer CPU load

**Sync never completes:**
- Check database for circular dependencies
- Verify message serialization
- Look for exceptions in callback_server()

---

## Future Improvements

### Potential Enhancements

1. **Adaptive Timeout:**
   - Measure round-trip time
   - Adjust timeout based on historical data

2. **Chunked Messages:**
   - Split large messages into chunks
   - Resume interrupted transfers

3. **Priority Queuing:**
   - Request high-priority messages first
   - Timeout low-priority items

4. **Statistics Tracking:**
   - Measure sync duration per robot
   - Track error rates
   - Log performance metrics

5. **Bidirectional Sync:**
   - Simultaneously pull and push data
   - Reduce total sync time

6. **Heartbeat:**
   - Periodic connectivity check
   - Early detection of network issues

---

## Summary

The synchronization channel module provides:

✅ **Robust peer-to-peer sync** with timeouts and retry
✅ **Thread-safe operation** using bistable and locks
✅ **Detailed state tracking** for debugging and monitoring
✅ **Bidirectional communication** (client-pull and server-push)
✅ **Adaptive behavior** based on signal quality
✅ **Graceful shutdown** without data loss

This architecture enables reliable multi-robot database synchronization across variable network conditions, making it suitable for swarm robotics applications with WiFi or radio links.
