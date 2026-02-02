# Phase 5: State Machine Documentation

## Overview

This document details the state machines governing the Acorn system:
1. **Vehicle Control States** - Overall operational modes
2. **Motor States** - Hardware connection status
3. **Autonomy State Machine** - Path following logic
4. **GPS Recording States** - Path recording modes

---

## 1. Vehicle Control States

### State Definitions

From `model.py`:

| State | Constant | Description |
|-------|----------|-------------|
| **Startup** | `CONTROL_STARTUP` | "Initializing..." |
| **GPS Startup** | `CONTROL_GPS_STARTUP` | "Waiting for GPS fix." |
| **Online** | `CONTROL_ONLINE` | "Online and awaiting commands." |
| **Autonomy** | `CONTROL_AUTONOMY` | "Autonomy operating." |
| **Autonomy Pause** | `CONTROL_AUTONOMY_PAUSE` | "Temporary autonomy pause." |
| **Low Voltage** | `CONTROL_LOW_VOLTAGE` | "Low voltage Pause." |
| **Distance Error** | `CONTROL_AUTONOMY_ERROR_DISTANCE` | "Autonomy failed - too far from path." |
| **Angle Error** | `CONTROL_AUTONOMY_ERROR_ANGLE` | "Autonomy failed - path angle too great." |
| **RTK Age Error** | `CONTROL_AUTONOMY_ERROR_RTK_AGE` | "Autonomy failed - rtk base data too old." |
| **Solution Age Error** | `CONTROL_AUTONOMY_ERROR_SOLUTION_AGE` | "Autonomy failed - gps solution too old." |
| **Override** | `CONTROL_OVERRIDE` | "Remote control override." |
| **Server Error** | `CONTROL_SERVER_ERROR` | "Server communication error." |
| **Motor Error** | `CONTROL_MOTOR_ERROR` | "Motor error detected." |
| **No Steering Solution** | `CONTROL_NO_STEERING_SOLUTION` | "No steering solution possible." |

### State Transition Diagram

```mermaid
stateDiagram-v2
    [*] --> STARTUP: Power On

    STARTUP --> GPS_STARTUP: Initialize complete
    GPS_STARTUP --> ONLINE: GPS fix acquired

    ONLINE --> AUTONOMY: activate_autonomy=true<br/>AND path loaded<br/>AND !autonomy_hold
    ONLINE --> OVERRIDE: Joystick activated

    AUTONOMY --> AUTONOMY_PAUSE: Path end reached<br/>(5 sec pause)
    AUTONOMY_PAUSE --> AUTONOMY: Pause complete

    AUTONOMY --> ONLINE: activate_autonomy=false
    AUTONOMY --> OVERRIDE: Joystick activated

    state ERROR_STATES {
        DISTANCE_ERROR: Too far from path
        ANGLE_ERROR: Angle too great
        RTK_AGE_ERROR: RTK data old
        SOLUTION_AGE_ERROR: GPS solution old
        LOW_VOLTAGE: Battery low
        SERVER_ERROR: Server timeout
        MOTOR_ERROR: Motor fault
        NO_STEERING: No solution
    }

    AUTONOMY --> DISTANCE_ERROR: distance > 2.5m
    AUTONOMY --> ANGLE_ERROR: angle > 120°
    AUTONOMY --> RTK_AGE_ERROR: RTK age > 20s
    AUTONOMY --> SOLUTION_AGE_ERROR: solution age > 1s

    DISTANCE_ERROR --> AUTONOMY: Error clears<br/>(after 60s retry)
    ANGLE_ERROR --> AUTONOMY: Error clears
    RTK_AGE_ERROR --> AUTONOMY: Error clears
    SOLUTION_AGE_ERROR --> AUTONOMY: Error clears

    OVERRIDE --> ONLINE: Joystick released<br/>AND autonomy_allowed
    OVERRIDE --> OVERRIDE: Joystick active

    ERROR_STATES --> ONLINE: autonomy_hold=true<br/>AND clear_autonomy_hold
```

### State Transition Logic

From `remote_control_process.py`:

#### Startup → Online
```python
# Initial state
self.control_state = model.CONTROL_STARTUP
self.autonomy_hold = True

# Transitions to ONLINE when:
# - GPS sample is valid
# - Motors not in error
# - No autonomy requested
if not self.activate_autonomy:
    self.control_state = model.CONTROL_ONLINE
```

#### Online → Autonomy
```python
# Requires all conditions:
# 1. activate_autonomy = True (from server command)
# 2. Path is loaded (nav_path.points not empty)
# 3. autonomy_hold = False (cleared by server or joystick)
# 4. No safety check failures

if self.activate_autonomy and not zero_output:
    self.control_state = model.CONTROL_AUTONOMY
```

#### Joystick Override
```python
if self.joy.activated():
    self.autonomy_hold = True
    self.activate_autonomy = False
    self.control_state = model.CONTROL_OVERRIDE

if self.joy.autonomy_allowed == False:
    self.control_state = model.CONTROL_OVERRIDE
```

#### Error State Transitions
```python
# Distance error
if abs(absolute_path_distance) > self.nav_path.maximum_allowed_distance_meters:
    zero_output = True
    self.control_state = model.CONTROL_AUTONOMY_ERROR_DISTANCE

# Angle error
if abs(gps_path_angle_error) > self.nav_path.maximum_allowed_angle_error_degrees:
    zero_output = True
    self.control_state = model.CONTROL_AUTONOMY_ERROR_ANGLE

# RTK age error
if self.gps.last_sample().rtk_age > _ALLOWED_RTK_AGE_SEC:  # 20s
    zero_output = True
    self.control_state = model.CONTROL_AUTONOMY_ERROR_RTK_AGE

# Solution age error
if solution_age > _ALLOWED_SOLUTION_AGE_SEC:  # 1s
    zero_output = True
    self.control_state = model.CONTROL_AUTONOMY_ERROR_SOLUTION_AGE
```

---

## 2. Motor States

### State Definitions

From `model.py`:

| State | Value | String | Description |
|-------|-------|--------|-------------|
| **Disconnected** | `0` | "Not connected." | CAN bus not responding |
| **Disabled** | `1` | "Motor error." | Connected but not enabled |
| **Enabled** | `2` | "Motors enabled." | Ready for commands |

### State Transition Diagram

```mermaid
stateDiagram-v2
    [*] --> DISCONNECTED: Power On

    DISCONNECTED --> DISABLED: CAN ping success<br/>AND voltage > 15V
    DISABLED --> ENABLED: Steering homing complete<br/>AND motion_allowed=true
    ENABLED --> DISABLED: motion_allowed=false<br/>OR thermal shutdown
    DISABLED --> DISCONNECTED: CAN timeout
    ENABLED --> DISCONNECTED: CAN timeout

    note right of DISCONNECTED
        motors_can.py attempts
        ping every 2 seconds
    end note

    note right of DISABLED
        Waiting for:
        - Steering initialization
        - Voltage OK
        - No thermal fault
    end note

    note right of ENABLED
        Accepting steering
        and velocity commands
    end note
```

### State Transition Logic

From `motors_can.py`:

```python
# Initial state
self.motors_connected = False
self.motors_initialized = False

# DISCONNECTED → DISABLED
def run_main():
    while True:
        if not input_voltage_okay:
            voltage = self.voltage_monitor.return_one_sample()
            input_voltage_okay = voltage > _SAFE_STARTUP_VOLTAGE  # 15V
            self.communicate_message(model.MOTOR_DISABLED)
            continue

        if not self.motors_connected:
            self.connect_to_motors()  # CAN ping
            if self.motors_connected:
                # State is now DISABLED

        elif not self.motors_initialized:
            self.initialize_motors()  # Steering homing
            # State transitions to ENABLED when complete

# DISABLED → ENABLED
state_to_send = model.MOTOR_ENABLED
for drive in self.motors:
    if not drive.controller.motion_allowed:
        state_to_send = model.MOTOR_DISABLED
self.communicate_message(state_to_send)
```

### Motor State in Shared Memory

```python
# motors_can.py writes state to shared memory
send_vals = np.array([
    [state, model.CLEAR_TO_WRITE, 0, 0],  # state = MOTOR_*
    ...
])
self.motor_output_values[:] = send_vals[:]

# remote_control_process.py reads state
self.motor_state = int(motor_message[0][0])
```

---

## 3. Autonomy State Machine

### Autonomy Hold Logic

The `autonomy_hold` flag prevents autonomy from activating. It must be explicitly cleared.

```mermaid
stateDiagram-v2
    [*] --> HOLD_ACTIVE: Startup<br/>(autonomy_hold=true)

    HOLD_ACTIVE --> HOLD_CLEARED: clear_autonomy_hold=true<br/>(from server)
    HOLD_ACTIVE --> HOLD_CLEARED: Joystick autonomy request

    HOLD_CLEARED --> HOLD_ACTIVE: Joystick activated
    HOLD_CLEARED --> HOLD_ACTIVE: Safety error (fatal)
    HOLD_CLEARED --> HOLD_ACTIVE: No steering solution

    state HOLD_CLEARED {
        [*] --> READY
        READY --> OPERATING: activate_autonomy=true
        OPERATING --> PAUSED: Path end / error
        PAUSED --> OPERATING: Pause complete
        OPERATING --> READY: activate_autonomy=false
    }
```

### Autonomy Activation Sequence

```mermaid
sequenceDiagram
    participant USER as Web UI
    participant SRV as Server
    participant MAIN as main_process
    participant RC as remote_control

    USER->>SRV: Load path
    SRV->>MAIN: RobotCommand.load_path = "path_key"
    MAIN->>RC: RobotSubset with loaded_path

    RC->>RC: reload_path()<br/>Generate spline

    USER->>SRV: Clear autonomy hold
    SRV->>MAIN: RobotCommand.clear_autonomy_hold = true
    MAIN->>RC: RobotSubset.clear_autonomy_hold = true

    RC->>RC: autonomy_hold = false

    USER->>SRV: Enable autonomy (speed=0.2)
    SRV->>MAIN: RobotCommand.activate_autonomy = true
    MAIN->>RC: RobotSubset.activate_autonomy = true

    RC->>RC: Wait 4 sec (alarm warning)
    RC->>RC: control_state = CONTROL_AUTONOMY
    RC->>RC: Begin path following
```

### Path Following State Machine

```mermaid
stateDiagram-v2
    [*] --> WAIT_PATH: No path loaded

    WAIT_PATH --> PATH_LOADED: Path received

    PATH_LOADED --> WAIT_AUTONOMY: autonomy_hold=true
    PATH_LOADED --> ALARM_WARNING: autonomy enabled<br/>AND !autonomy_hold

    ALARM_WARNING --> FOLLOWING: 4 sec elapsed

    FOLLOWING --> END_PAUSE: Distance to end < 1m<br/>AND angle < 45°
    END_PAUSE --> PATH_COMPLETE: 5 sec pause complete

    PATH_COMPLETE --> FOLLOWING: repeat_path=true
    PATH_COMPLETE --> NEXT_ROW: Multi-row path
    PATH_COMPLETE --> WAIT_AUTONOMY: Single path complete

    NEXT_ROW --> FOLLOWING: Load next row

    FOLLOWING --> ERROR_PAUSE: Safety check failed
    ERROR_PAUSE --> FOLLOWING: 60 sec retry<br/>(non-fatal error)
    ERROR_PAUSE --> WAIT_AUTONOMY: Fatal error

    state FOLLOWING {
        [*] --> READ_GPS
        READ_GPS --> CALC_ERROR: GPS valid
        CALC_ERROR --> PID_CONTROL
        PID_CONTROL --> SAFETY_CHECK
        SAFETY_CHECK --> SEND_MOTORS: All checks pass
        SEND_MOTORS --> READ_GPS
    }
```

### Safety Check Decision Tree

```mermaid
flowchart TB
    START[Safety Checks] --> V{Voltage > 25V?}
    V -->|No| LOW_V[CONTROL_LOW_VOLTAGE<br/>zero_output=true]
    V -->|Yes| D{Distance < 2.5m?}

    D -->|No| DIST_E[CONTROL_AUTONOMY_ERROR_DISTANCE<br/>zero_output=true]
    D -->|Yes| A{Angle < 120°?}

    A -->|No| ANG_E[CONTROL_AUTONOMY_ERROR_ANGLE<br/>zero_output=true]
    A -->|Yes| R{RTK age < 20s?}

    R -->|No| RTK_E[CONTROL_AUTONOMY_ERROR_RTK_AGE<br/>zero_output=true]
    R -->|Yes| S{Solution age < 1s?}

    S -->|No| SOL_E[CONTROL_AUTONOMY_ERROR_SOLUTION_AGE<br/>zero_output=true]
    S -->|Yes| SRV{Server comms < 60s?}

    SRV -->|No| SRV_E[Server timeout<br/>zero_output=true]
    SRV -->|Yes| OK[All checks passed<br/>Continue autonomy]
```

---

## 4. GPS Recording States

### State Definitions

From `model.py`:

| State | Constant | Description |
|-------|----------|-------------|
| **Clear** | `GPS_RECORDING_CLEAR` | No recording, buffer cleared |
| **Record** | `GPS_RECORDING_ACTIVATE` | Actively recording GPS points |
| **Pause** | `GPS_RECORDING_PAUSE` | Recording paused, buffer retained |

### State Transition Diagram

```mermaid
stateDiagram-v2
    [*] --> CLEAR: Default

    CLEAR --> RECORD: User: "Record"
    RECORD --> PAUSE: User: "Pause"
    PAUSE --> RECORD: User: "Record"
    PAUSE --> CLEAR: User: "Clear"
    RECORD --> CLEAR: User: "Clear"
```

---

## 5. Joystick State Machine

### Joystick Activation Logic

From `remote_control_process.py`:

```mermaid
stateDiagram-v2
    [*] --> DISCONNECTED: Startup

    DISCONNECTED --> SAFETY_CHECK_1: Initial value changed

    SAFETY_CHECK_1 --> SAFETY_CHECK_2: Switches confirmed DOWN
    note right of SAFETY_CHECK_1
        Waiting for operator to
        move switches to down position
    end note

    SAFETY_CHECK_2 --> READY: Switches confirmed UP
    note right of SAFETY_CHECK_2
        Waiting for operator to
        move switches to up position
    end note

    READY --> ACTIVE: Throttle/steer input detected
    ACTIVE --> READY: Input released
    ACTIVE --> AUTONOMY_REQUEST: Autonomy switch toggled
```

### Joystick Override Behavior

```python
# From remote_control_process.py

# Joystick takes immediate control
if self.joy.activated():
    self.logger.info("DISABLED AUTONOMY because joystick is activated")
    self.autonomy_hold = True
    self.activate_autonomy = False
    self.control_state = model.CONTROL_OVERRIDE

# Autonomy not allowed when switch is in wrong position
if self.joy.autonomy_allowed == False:
    self.autonomy_hold = True
    self.activate_autonomy = False
    self.control_state = model.CONTROL_OVERRIDE

# Joystick can request autonomy
elif self.joy.autonomy_requested == True:
    self.gps.flush_serial()
    self.autonomy_hold = False
    self.activate_autonomy = True
    self.joy.autonomy_requested = False
```

---

## 6. System Manager Automation (Server)

### Fleet Automation State Machine

From `system_manager.py`:

```mermaid
stateDiagram-v2
    [*] --> MONITORING: Startup

    MONITORING --> SETTLING: Robot comes online<br/>(data_age < 5s)

    SETTLING --> READY: 30 sec elapsed
    note right of SETTLING
        Waiting for system
        to stabilize
    end note

    READY --> CHECK_HOLD: autonomy_hold=true?

    CHECK_HOLD --> CLEAR_HOLD: voltage > 40V<br/>AND time < 10 hours
    CHECK_HOLD --> WAIT_CHARGE: voltage < 40V
    note right of WAIT_CHARGE
        "Could resume autonomy but
        voltage below threshold"
    end note

    CLEAR_HOLD --> ENABLE_AUTONOMY: clear_autonomy_hold sent

    ENABLE_AUTONOMY --> MONITORING: autonomy enabled
    note right of ENABLE_AUTONOMY
        Only if control_state
        == CONTROL_ONLINE
    end note

    MONITORING --> OFFLINE: data_age > 3600s
    OFFLINE --> MONITORING: Robot reconnects
```

### Automation Thresholds

```python
# From system_manager.py

AUTONOMY_AT_STARTUP = True
AUTONOMY_SPEED = 0.2  # m/s

_ONLINE_DATA_AGE_MAXIMUM_SEC = 5        # Fresh data threshold
_OFFLINE_DATA_AGE_MINIMUM_SEC = 3600    # 1 hour = offline
_ONLINE_SETTLING_TIME_SEC = 30          # Wait before enabling
_RESUME_AUTONOMY_MINIMUM_VOLTAGE = 40   # Voltage threshold
_MAXIMUM_TIME_TO_ATTEMPT_AUTONOMY_SEC = 36000  # 10 hours
_CLEAR_AUTONOMY_HOLD_COMMAND_OBJECT_TIME_SEC = 5
_LOOP_DELAY_SEC = 2
```

---

## 7. State Summary Table

| Component | States | Primary Transitions |
|-----------|--------|---------------------|
| **Control State** | 14 states | Online ↔ Autonomy ↔ Error states |
| **Motor State** | 3 states | Disconnected → Disabled → Enabled |
| **Autonomy Hold** | 2 states | Hold ↔ Cleared |
| **GPS Recording** | 3 states | Clear ↔ Record ↔ Pause |
| **Joystick** | 5 states | Disconnected → Safety → Ready → Active |
| **System Manager** | 6 states | Monitoring → Settling → Enable |

---

## 8. Key Timing Constants

| Constant | Value | Purpose |
|----------|-------|---------|
| `_RESUME_MOTION_WARNING_TIME_SEC` | 4 sec | Alarm before autonomy starts |
| `_PATH_END_PAUSE_SEC` | 5 sec | Pause at path end |
| `_DISENGAGEMENT_RETRY_DELAY_SEC` | 60 sec | Wait after non-fatal error |
| `_ALLOWED_RTK_AGE_SEC` | 20 sec | Max RTK correction age |
| `_ALLOWED_SOLUTION_AGE_SEC` | 1 sec | Max GPS solution age |
| `SERVER_COMMUNICATION_DELAY_LIMIT_SEC` | 10 sec | Server timeout |
| `_ONLINE_SETTLING_TIME_SEC` | 30 sec | System manager settling |

---

## Next Steps

Phase 6 will create a configuration inventory:
- All hardcoded constants
- Default values
- Safety thresholds
- Tuning parameters
