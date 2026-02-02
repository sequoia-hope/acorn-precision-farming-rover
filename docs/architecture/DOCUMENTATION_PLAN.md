# Acorn Architecture Documentation Plan

## Goal
Build a systematic, diagrammatic understanding of the Acorn precision farming rover system, focusing on the `vehicle/` and `server/` components, in preparation for refactoring into a generalized robot framework.

---

## Phase 1: System Context & Boundaries

**Objective**: Establish what the system is, what it interfaces with, and its deployment topology.

### Deliverables:
1. **System Context Diagram** (C4 Level 1)
   - Acorn system as a whole
   - External actors: Operator (web UI), GPS satellites, RTK base station
   - Physical boundaries: Vehicle hardware, Server infrastructure

2. **Deployment Diagram**
   - Docker containers (vehicle, server)
   - Hardware mapping (Jetson/RPi, CAN bus, GPS receivers)
   - Network topology (ZMQ ports, HTTP, Redis)

### Source Files to Review:
- `docker-compose.yml`, `Dockerfile.*`
- `run_docker_*.sh` scripts
- `vehicle/server_config.yaml`, `vehicle/server_config_sim.yaml`

---

## Phase 2: Container/Component Architecture

**Objective**: Document the major components within vehicle and server subsystems.

### Deliverables:
1. **Vehicle Component Diagram** (C4 Level 2)
   - Process boundaries (main, motors, remote_control, rtk, server_comms)
   - Inter-process communication (pipes, shared memory)
   - Hardware interfaces (CAN, GPIO, I2C, serial)

2. **Server Component Diagram** (C4 Level 2)
   - Service boundaries (Flask, ZMQ workers, Redis, system_manager)
   - API surface (HTTP endpoints, ZMQ message types)
   - Data stores (Redis key structure)

### Source Files to Review:
- `vehicle/main_process.py` - process orchestration
- `vehicle/remote_control_process.py` - autonomy subprocess
- `vehicle/motor_controller.py` - motor subprocess
- `server/autolaunch_server.sh` - server process topology
- `server/server.py` - HTTP API
- `server/zmq_server_pirate.py` - ZMQ broker

---

## Phase 3: Data Flow Diagrams

**Objective**: Trace how data moves through the system for key operations.

### Deliverables:
1. **Robot State Flow Diagram**
   - GPS data: receivers → rtk_process → gps.py → robot.location
   - Motor state: CAN bus → motors_can.py → robot.motor_state
   - Aggregation in main_process → server → Redis → web UI

2. **Command Flow Diagram**
   - User action → Flask API → Redis → vehicle polling → remote_control
   - Command types: load_path, activate_autonomy, record_gps

3. **Autonomy Control Loop Diagram**
   - Path loading and spline generation
   - GPS position → error calculation → PID → steering → motors
   - Error handling and recovery states

### Source Files to Review:
- `vehicle/model.py` - Robot, RobotCommand, RobotSubset classes
- `vehicle/server_comms.py` - ZMQ protocol
- `vehicle/gps.py`, `vehicle/rtk_process.py` - GPS pipeline
- `vehicle/steering.py` - kinematics
- `server/redis_utils.py` - data storage patterns

---

## Phase 4: Interface Documentation

**Objective**: Document the contracts between components.

### Deliverables:
1. **IPC Interface Table**
   | Interface | From | To | Mechanism | Data Format |
   |-----------|------|-----|-----------|-------------|
   | ... | ... | ... | ... | ... |

2. **ZMQ Protocol Specification**
   - Message types (_CMD_* constants)
   - Request/response patterns
   - Serialization format (pickle)

3. **HTTP API Reference**
   - Endpoints, methods, parameters
   - Response formats

4. **CAN Bus Protocol Summary**
   - Message IDs, data formats
   - Motor command structure

### Source Files to Review:
- `vehicle/main_process.py` - _CMD_* constants, pipe communications
- `vehicle/motors_can.py`, `vehicle/corner_actuator_can.py` - CAN protocol
- `server/server.py` - Flask routes
- `server/zmq_server_pirate.py` - ZMQ message handling

---

## Phase 5: State Machine Documentation

**Objective**: Document the behavioral logic of the system.

### Deliverables:
1. **Vehicle Control State Diagram**
   - States: STARTUP, GPS_STARTUP, ONLINE, AUTONOMY, ERROR_*
   - Transitions and guards
   - Recovery paths

2. **Motor State Diagram**
   - DISCONNECTED → DISABLED → ENABLED
   - Transition triggers

3. **Autonomy State Diagram**
   - Manual mode, path loading, path following, completion
   - Error conditions and recovery

### Source Files to Review:
- `vehicle/model.py` - CONTROL_*, MOTOR_*, GPS_RECORDING_* constants
- `vehicle/remote_control_process.py` - state machine logic

---

## Phase 6: Configuration & Constants Inventory

**Objective**: Identify all tunable parameters scattered through the codebase.

### Deliverables:
1. **Configuration Inventory Table**
   | Parameter | Current Value | File | Line | Description |
   |-----------|---------------|------|------|-------------|
   | ... | ... | ... | ... | ... |

2. **Categories**:
   - Physical constants (wheel base, radius)
   - Timing/rates (update periods, timeouts)
   - Control gains (PID parameters)
   - Safety thresholds (voltage, RTK age, distance limits)
   - Network configuration (ports, addresses)

### Source Files to Review:
- `vehicle/remote_control_process.py` - many hardcoded values
- `vehicle/steering.py` - geometry constants
- `vehicle/gps.py` - timing parameters
- `server/system_manager.py` - automation thresholds

---

## Diagram Format Recommendations

### Tool Options:
1. **Mermaid** (Markdown-embeddable, git-friendly)
   - Good for: flowcharts, sequence diagrams, state diagrams
   - Rendered in GitHub, VS Code, many doc systems

2. **PlantUML** (Text-based, detailed)
   - Good for: component diagrams, deployment diagrams
   - More expressive than Mermaid

3. **Draw.io / diagrams.net** (Visual editor)
   - Good for: complex layouts, custom styling
   - Export to SVG/PNG for docs

### Recommended Approach:
- Use **Mermaid** for version-controlled diagrams in markdown
- Create diagrams alongside code documentation
- Store in `docs/architecture/` directory

---

## Documentation Structure

```
docs/architecture/
├── DOCUMENTATION_PLAN.md          (this file)
├── 01-system-context.md           (Phase 1)
│   └── diagrams/
│       ├── context.mermaid
│       └── deployment.mermaid
├── 02-components.md               (Phase 2)
│   └── diagrams/
│       ├── vehicle-components.mermaid
│       └── server-components.mermaid
├── 03-data-flows.md               (Phase 3)
│   └── diagrams/
│       ├── robot-state-flow.mermaid
│       ├── command-flow.mermaid
│       └── autonomy-loop.mermaid
├── 04-interfaces.md               (Phase 4)
│   ├── ipc-interfaces.md
│   ├── zmq-protocol.md
│   ├── http-api.md
│   └── can-protocol.md
├── 05-state-machines.md           (Phase 5)
│   └── diagrams/
│       ├── control-states.mermaid
│       ├── motor-states.mermaid
│       └── autonomy-states.mermaid
└── 06-configuration.md            (Phase 6)
    └── config-inventory.csv
```

---

## Execution Order

| Phase | Estimated Scope | Dependencies |
|-------|-----------------|--------------|
| 1. System Context | Broad overview | None |
| 2. Components | Medium detail | Phase 1 |
| 3. Data Flows | Detailed tracing | Phase 2 |
| 4. Interfaces | Reference docs | Phase 2, 3 |
| 5. State Machines | Behavioral logic | Phase 3 |
| 6. Configuration | Inventory task | All phases |

**Recommended approach**: Work through phases sequentially, as each builds understanding for the next. Each phase can be reviewed before proceeding.

---

## Questions to Resolve During Documentation

1. **GPS Dual-receiver logic**: How exactly is heading calculated from two GPS receivers?
2. **RTK integration**: What's the relationship between rtkrcv binary and pyubx2?
3. **Path representation**: How are paths stored, interpolated, and followed?
4. **Energy segments**: What are they tracking and how are they used?
5. **System manager automation**: What conditions trigger automatic autonomy?
6. **Simulation mode**: How complete is the simulation vs. hardware mode divergence?

---

## Next Steps

To begin Phase 1, we would:
1. Read the Docker/deployment configurations in detail
2. Create the system context diagram showing actors and boundaries
3. Create the deployment diagram showing container topology
4. Document in `docs/architecture/01-system-context.md`

Ready to proceed when you are.
