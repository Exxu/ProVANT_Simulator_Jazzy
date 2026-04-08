# provant_step_manager_system

This module contains the `provant_step_manager_system` Gazebo Harmonic system plugin responsible for the **observation side** of ProVANT's temporal semantics.

Its role is to observe the real simulation progression inside Gazebo Harmonic and expose that progression back to ROS 2 through:

- `/provant_simulator/step_clock`
- `/provant_simulator/reset`

It is the direct replacement for the **temporal observation responsibilities** of the legacy `step_manager` from the Gazebo Classic architecture.

---

## Goal

The goal of `provant_step_manager_system` is to provide the **observable simulation clock** and **reset notification** required by the simulator temporal loop.

More specifically, it is responsible for:

- observing simulation iterations in Gazebo Harmonic
- publishing `/provant_simulator/step_clock`
- detecting world reset events
- notifying ROS 2 through `/provant_simulator/reset`
- publishing a valid post-reset `step_clock`

This plugin does **not** decide when the simulation should advance. That responsibility belongs to `simulation_manager` together with `provant_step_command_bridge`.

---

## Architectural role

In the migrated architecture, the temporal loop is split into three separate pieces:

- `simulation_manager`
  - decides when the simulator is allowed to advance

- `provant_step_command_bridge`
  - translates ROS 2 step requests into Gazebo world control commands

- `provant_step_manager_system`
  - observes the resulting simulation step and publishes temporal feedback

This separation is intentional.

In Gazebo Classic, the old `step_manager` both triggered stepping and observed the resulting simulation clock.

In Gazebo Harmonic, these concerns are separated:

- **command side** -> `provant_step_command_bridge`
- **observation side** -> `provant_step_manager_system`

---

## Why this module is a Gazebo system plugin

`provant_step_manager_system` is implemented as a **Gazebo system plugin** because it must run inside the simulation lifecycle.

It needs direct access to:

- `Configure`
- `PreUpdate`
- `Reset`

These hooks allow the plugin to observe:

- current iteration count
- current simulation time
- world reset events

This is not something a plain ROS 2 node can observe with the same fidelity.

---

## Package structure

```text
step_manager_system/
├── CMakeLists.txt
├── include/
│   └── provant_step_manager_system/
│       └── step_manager_system.hpp
├── src/
│   └── step_manager_system.cpp
└── ...
```

The exact folder layout may vary depending on how the plugin is placed inside `provant_simulator_gz_plugins`, but the module is conceptually organized as a single plugin with its header and implementation.

---

## Components

### 1. Header

File:

```text
include/provant_step_manager_system/step_manager_system.hpp
```

This file declares the `StepManagerSystem` class.

It contains:

- Gazebo system interfaces
- ROS 2 publisher and client declarations
- internal synchronization state
- temporal publication state
- reset notification state

---

### 2. Implementation

File:

```text
src/step_manager_system.cpp
```

This file implements the plugin behavior.

Its main responsibilities are:

- configure the ROS 2 node embedded in the plugin
- publish `/provant_simulator/step_clock`
- detect reset events in `Reset(...)`
- notify ROS 2 through `/provant_simulator/reset`
- guarantee a post-reset `step_clock`

---

## Communication contract

### Output topic

The plugin publishes:

```text
/provant_simulator/step_clock
```

Type:

```text
provant_simulator_interfaces/msg/StepClock
```

This message represents the observable temporal state of the simulator.

It contains:

- `step`
- `time`

where:

- `step` corresponds to the observed simulation iteration
- `time` corresponds to the observed simulation time

---

### Reset notification service

The plugin calls:

```text
/provant_simulator/reset
```

Type:

```text
std_srvs/srv/Empty
```

This service is used to notify the ROS 2 side that Gazebo reset the world.

---

## How it works

The plugin operates mainly in two phases:

### 1. Normal simulation observation

During `PreUpdate(...)`, the plugin reads the Gazebo update information and publishes a `StepClock` message whenever a new iteration is observed.

This is the regular temporal feedback path.

The published `step_clock` corresponds to the temporal state visible at the beginning of the current simulation iteration.

That makes the behavior consistent with the intended simulator temporal contract, where the control side reacts to the next discrete simulation instant after a released step.

### 2. Reset observation

During `Reset(...)`, the plugin:

- clears the previously published iteration state
- schedules a reset notification to ROS 2
- marks that a first post-reset clock must be published

Then, in the next useful `PreUpdate(...)`, it:

- sends the reset notification
- forces publication of a post-reset `step_clock`

This guarantees that the reset becomes observable from ROS 2.

---

## Why post-reset publication matters

Without an explicit post-reset publication, the reset could be detected internally but not become clearly visible to the ROS 2 side.

That would leave an ambiguity in the simulator timing loop.

To avoid that, `provant_step_manager_system` guarantees that after reset there is a new valid temporal observation published on:

```text
/provant_simulator/step_clock
```

This is essential to preserve the temporal semantics expected by the original simulator.

---

## Temporal semantics of `step_clock`

`provant_step_manager_system` does not request steps.

Instead, it reports the temporal state observed inside Gazebo after the simulator control side has released progression.

In the migrated architecture, the intended sequence is:

```text
control group ready
-> simulation_manager publishes /provant_simulator/step
-> provant_step_command_bridge requests one simulation step
-> Gazebo advances
-> provant_step_manager_system publishes /provant_simulator/step_clock in PreUpdate
```

This means the published `step_clock` is the observable temporal reference used by the control side to process the next discrete simulation instant.

---

## Parameters

The plugin reads configuration from the SDF plugin block.

Typical parameters are:

### `ros_node_name`

Default:

```text
provant_step_manager_system
```

Defines the ROS 2 node name created inside the plugin.

### `step_clock_topic`

Default:

```text
/provant_simulator/step_clock
```

Defines the ROS 2 topic where `StepClock` will be published.

### `reset_service`

Default:

```text
/provant_simulator/reset
```

Defines the ROS 2 service called when the world is reset.

---

## Relationship with other packages

`provant_step_manager_system` is not a standalone timing controller.

It works together with:

- `simulation_manager`
- `provant_step_command_bridge`

Their responsibilities are:

- `simulation_manager`
  - decides when the simulator may advance

- `provant_step_command_bridge`
  - requests one discrete step from Gazebo

- `provant_step_manager_system`
  - publishes the observable simulation clock and reset notification

Together, they close the simulator temporal loop in the migrated architecture.

## Expected startup condition

For correct simulator behavior, the Gazebo world is expected to start **paused**.

This is especially important for aerial vehicles, where even a brief uncontrolled free-running interval can produce unwanted motion before the control and synchronization loop is ready.

The plugin itself does not enforce startup pause, but it is designed to operate inside a simulator architecture where:

- the world starts paused
- the world only advances when discrete step requests are issued

---

## Example SDF plugin block

A typical SDF configuration block looks like this:

```xml
<plugin
  filename="libprovant_step_manager_system.so"
  name="provant::simulator::StepManagerSystem">
  <ros_node_name>provant_step_manager_system</ros_node_name>
  <step_clock_topic>/provant_simulator/step_clock</step_clock_topic>
  <reset_service>/provant_simulator/reset</reset_service>
</plugin>
```

This is the standard way to attach the plugin to a Gazebo Harmonic world.

---

## How to build

From the workspace root:

```bash
colcon build --packages-select provant_simulator_gz_plugins
```

Then source the workspace:

```bash
source install/setup.bash
```

---

## How to use

`provant_step_manager_system` is not run directly as a ROS 2 executable.

It is loaded by Gazebo when the world SDF includes the corresponding plugin block.

A typical usage pattern is:

1. launch Gazebo Harmonic server with a world containing the plugin
2. let the plugin configure itself inside the world
3. observe `/provant_simulator/step_clock` from ROS 2
4. provide `/provant_simulator/reset` so reset notifications can be propagated

---

## Manual validation

A simple manual validation flow is:

1. start Gazebo with a world that loads `provant_step_manager_system`
2. observe:

```bash
ros2 topic echo /provant_simulator/step_clock
```

3. trigger a reset on the world
4. verify that:
   - `/provant_simulator/reset` is called
   - a new post-reset `step_clock` is published

---

## Expected behavior

During normal operation:

- the plugin publishes the current simulation step
- the plugin publishes the current simulation time

During reset:

- the plugin detects the reset
- the plugin notifies ROS 2
- the plugin publishes a valid post-reset clock

---

## Failure conditions

The plugin should be considered unhealthy if:

- no `step_clock` is published during simulation
- reset events are not observed
- `/provant_simulator/reset` is never called
- no valid post-reset `step_clock` is published

These are strong indicators that temporal observation is broken.

---

## Scope of migration

This module is part of the migration from:

- ROS 2 Iron + Gazebo Classic

to:

- ROS 2 Jazzy + Gazebo Harmonic

Its specific contribution to that migration is to reintroduce the **temporal observation side** of the old `step_manager`.

It does not by itself replace the full legacy `step_manager`. Instead, it replaces the observation and reset-notification half of that behavior.

---