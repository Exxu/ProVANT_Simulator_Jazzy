# provant_simulator_step_command_bridge

This package contains the `provant_step_command_bridge` node, which is responsible for translating ROS 2 step requests into Gazebo Harmonic world control commands.

Its role in the simulator is to preserve the temporal behavior that, in the original Gazebo Classic architecture, was achieved through direct calls to `world->Step(1)`.

In the new architecture for **ROS 2 Jazzy** and **Gazebo Harmonic**, this responsibility is separated from temporal observation and implemented as a dedicated ROS 2 node.

---

## Goal

The goal of `provant_step_command_bridge` is to provide the **command-side temporal bridge** between the ProVANT simulator and Gazebo Harmonic.

More specifically, it is responsible for:

- subscribing to `/provant_simulator/step`
- receiving requests for discrete simulation advancement
- sending a Gazebo world control request to advance the simulation
- preserving a single-step execution model compatible with the original simulator semantics

This package does **not** observe simulation time directly. That responsibility belongs to `provant_step_manager_system`.

---

## Architectural role

In the migrated architecture, the temporal loop is split into three separate pieces:

- `simulation_manager`
  - decides when the simulator is allowed to advance

- `provant_step_command_bridge`
  - translates the ROS 2 step request into a Gazebo Harmonic world control request

- `provant_step_manager_system`
  - observes the actual simulation progress and publishes `/provant_simulator/step_clock`

This separation is intentional.

In Gazebo Classic, the old `step_manager` both triggered the simulation step and observed the resulting time progression.

In Gazebo Harmonic, these two concerns are separated:

- **command side** -> `provant_step_command_bridge`
- **observation side** -> `provant_step_manager_system`

---

## Why this package is a ROS 2 node and not a Gazebo plugin

`provant_step_command_bridge` is implemented as a **ROS 2 node**, not as a Gazebo system plugin.

This is because its job is not to run inside the simulation update loop.

Instead, it only needs to:

- listen to a ROS 2 topic
- build a Gazebo world control request
- send that request to the world

That makes it a clean bridge between ROS 2 and Gazebo transport.

This also gives several advantages:

- simpler separation of responsibilities
- easier testing
- lower coupling with Gazebo internals
- cleaner integration with `simulation_manager`

---

## Package structure

```text
provant_simulator_step_command_bridge/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── provant_step_command_bridge/
│       └── step_command_bridge.hpp
├── src/
│   ├── step_command_bridge.cpp
│   └── step_command_bridge_node.cpp
└── launch/
    └── step_command_bridge.launch.py
```

---

## Components

### 1. Header

File:

```text
include/provant_step_command_bridge/step_command_bridge.hpp
```

This file declares the `StepCommandBridge` class.

It contains:

- ROS 2 node definition
- subscriber declaration
- Gazebo transport node declaration
- internal state used to serialize step requests

---

### 2. Implementation

File:

```text
src/step_command_bridge.cpp
```

This file implements the `StepCommandBridge` class.

Its main responsibilities are:

- declare and read node parameters
- subscribe to `/provant_simulator/step`
- build a `gz::msgs::WorldControl` request
- send that request to `/world/<world_name>/control`

---

### 3. Executable entry point

File:

```text
src/step_command_bridge_node.cpp
```

This file contains only the `main()` function.

It initializes ROS 2, creates the node instance, spins it, and shuts ROS 2 down when the process exits.

This separation keeps the package aligned with the style used across the simulator.

---

### 4. Launch file

File:

```text
launch/step_command_bridge.launch.py
```

This launch file starts the node and allows the world name to be configured through a launch argument.

The step topic is intentionally kept fixed as:

```text
/provant_simulator/step
```

because it is part of the simulator's temporal contract.

---

## Communication contract

### Input topic

The node subscribes to:

```text
/provant_simulator/step
```

Type:

```text
std_msgs/msg/Empty
```

Each received message means:

> advance the simulation by one discrete step

---

### Gazebo backend endpoint

The node sends requests to:

```text
/world/<world_name>/control
```

using Gazebo transport and the message type:

```text
gz.msgs.WorldControl
```

The expected response type is:

```text
gz.msgs.Boolean
```

---

## How it works

For each message received on:

```text
/provant_simulator/step
```

the node builds a world control request with:

- `pause = true`
- `multi_step = 1`

This means:

- the simulation must remain paused as a controlled system
- Gazebo must execute exactly one simulation step

Conceptually, this reproduces the role that `world->Step(1)` had in Gazebo Classic.

---

## Why `multi_step = 1` is used

Gazebo Harmonic's `WorldControl` message supports both:

- `step`
- `multi_step`

For ProVANT, `multi_step = 1` is preferred because it explicitly communicates the required semantics:

- exactly one discrete step

This is more explicit than using a boolean `step = true`, and it maps more directly to the legacy idea of:

```text
Step(1)
```

The command used by the node is therefore equivalent to:

- keep the world paused
- execute one step

---

## Internal protection against overlapping requests

The node keeps an internal flag:

```text
requestInFlight_
```

This prevents a new step request from being processed while another Gazebo control request is still in progress.

If a new `/provant_simulator/step` message arrives while the previous request has not finished yet, the node logs a warning and ignores the overlapping request.

This protects the simulator against accidental reentrancy or request piling.

---

## Parameters

### `world_name`

Type:

```text
string
```

Default:

```text
default
```

This parameter determines which Gazebo world control service will be used:

```text
/world/<world_name>/control
```

It should match the name of the world defined in the SDF.

---

### `timeout_ms`

Type:

```text
integer
```

Default:

```text
3000
```

This parameter defines how long the node will wait for the Gazebo control request to complete.

---

## What this package does not do

This package does **not**:

- publish simulation time
- publish `/provant_simulator/step_clock`
- detect world reset
- notify `/provant_simulator/reset`
- decide when the simulation should advance

Those responsibilities belong to other components:

- `simulation_manager`
- `provant_step_manager_system`

---

## Relationship with `simulation_manager`

`simulation_manager` remains the temporal coordinator of the simulator.

Its role is to decide when all registered groups are ready and when the simulator is allowed to advance.

When that happens, it publishes:

```text
/provant_simulator/step
```

`provant_step_command_bridge` receives that message and performs the actual request to Gazebo.

So the temporal command flow is:

```text
simulation_manager
  -> /provant_simulator/step
provant_step_command_bridge
  -> /world/<world_name>/control
Gazebo Harmonic
  -> executes one simulation step
```

---

## Relationship with `provant_step_manager_system`

After Gazebo performs the requested step, `provant_step_manager_system` observes the new simulation state and publishes:

```text
/provant_simulator/step_clock
```

So the full temporal loop becomes:

```text
simulation_manager
  -> /provant_simulator/step
provant_step_command_bridge
  -> Gazebo world control request
Gazebo Harmonic
  -> advances one step
provant_step_manager_system
  -> /provant_simulator/step_clock
```

This closes the temporal cycle.

---

## Expected startup condition

For correct simulator behavior, the Gazebo world is expected to start **paused**.

This is a strong requirement in ProVANT because vehicles may spawn in unstable configurations, and allowing the world to run even briefly before the controllers and synchronization loop are ready can produce unwanted motion.

Therefore, `provant_step_command_bridge` is designed under the assumption that:

- the world starts paused
- the world only advances when `/provant_simulator/step` is published

---

## How to build

From the workspace root:

```bash
colcon build --packages-select provant_simulator_step_command_bridge
```

Then source the workspace:

```bash
source install/setup.bash
```

---

## How to run

Run the node with:

```bash
ros2 launch provant_simulator_step_command_bridge step_command_bridge.launch.py
```

You can choose the world name explicitly:

```bash
ros2 launch provant_simulator_step_command_bridge step_command_bridge.launch.py world_name:=provant_world
```

---

## Manual validation

A simple manual validation flow is:

1. Start Gazebo Harmonic server with a world
2. Ensure the world is paused
3. Start `provant_step_command_bridge`
4. Publish a message to `/provant_simulator/step`
5. Observe that the simulation advances by one step

Example command to publish one step:

```bash
ros2 topic pub --once /provant_simulator/step std_msgs/msg/Empty "{}"
```

---

## Expected behavior

For each message on `/provant_simulator/step`:

- one Gazebo world control request is sent
- the world advances by one step
- the world remains under paused/discrete control
- temporal observation is later reported by `provant_step_manager_system`

---

## Failure conditions

The node should be considered unhealthy if:

- the Gazebo world control service cannot be reached
- the service call times out
- Gazebo reports a failure response
- step requests arrive faster than they can be processed and are repeatedly dropped

These are integration issues that should be handled at the simulator orchestration level.

---

## Scope of migration

This package is part of the migration from:

- ROS 2 Iron + Gazebo Classic

to:

- ROS 2 Jazzy + Gazebo Harmonic

Its specific contribution to that migration is to reintroduce the **command side of discrete temporal control**.

It does not replace the full legacy `step_manager` by itself. Instead, it replaces the part of the old architecture that used to request simulation steps.

---

## Summary

`provant_step_command_bridge` is the ROS 2 node responsible for translating ProVANT temporal step requests into Gazebo Harmonic world control commands.

In practical terms, it:

- listens to `/provant_simulator/step`
- requests one world step from Gazebo
- preserves discrete temporal advancement
- works together with `simulation_manager` and `provant_step_manager_system` to close the simulator timing loop

This package is the command-side counterpart of the new temporal architecture in Jazzy/Harmonic.
