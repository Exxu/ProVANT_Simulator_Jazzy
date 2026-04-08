# provant_temporal_loop_tests

This package contains end-to-end integration tests for the ProVANT temporal loop running on **ROS 2 Jazzy** and **Gazebo Harmonic**.

Its goal is to validate that the migrated architecture preserves the temporal semantics expected by the simulator when using:

- `simulation_manager`
- `provant_step_command_bridge`
- `provant_step_manager_system`

---

## Goal

The goal of this package is to verify the complete temporal control loop implemented by the migrated simulator architecture.

More specifically, it validates that:

- a control group can register itself in `simulation_manager`
- `simulation_manager` only advances the simulator after all registered groups are ready
- a message on `/provant_simulator/step` results in exactly one requested simulation step
- `provant_step_manager_system` publishes `/provant_simulator/step_clock`
- the observed temporal state advances consistently by one simulation step
- the end-to-end discrete temporal loop remains consistent with the intended simulator semantics

---

## Architectural role

In the migrated architecture, the temporal loop is split into three separate pieces:

- `simulation_manager`
  - decides when the simulator is allowed to advance

- `provant_step_command_bridge`
  - translates ROS 2 step requests into Gazebo world control commands

- `provant_step_manager_system`
  - observes the resulting simulation progression and publishes `/provant_simulator/step_clock`

This test package exists to validate the interaction between those three components as a single closed loop.

---

## Temporal contract under test

The main temporal contract validated by this package is the following:

1. the simulation manager enters `running`
2. a registered control group publishes `/<group_namespace>/ready`
3. `simulation_manager` publishes `/provant_simulator/step`
4. `provant_step_command_bridge` sends a Gazebo world control request with one step
5. Gazebo advances exactly one iteration
6. `provant_step_manager_system` publishes `/provant_simulator/step_clock`
7. the reported temporal state reflects one discrete advancement:
   - `step = N + 1`
   - `time = T + dt`

Conceptually, the loop being tested is:

```text
control_group ready
-> simulation_manager publishes /provant_simulator/step
-> provant_step_command_bridge requests one Gazebo step
-> Gazebo advances one iteration
-> provant_step_manager_system publishes /provant_simulator/step_clock
-> control_group computes and becomes ready again
```

---

## What this package tests

This package tests the following observable properties:

- registration of a control group in `simulation_manager`
- transition of the simulation manager to the `running` state
- publication of `/provant_simulator/step`
- observation of `/provant_simulator/step_clock`
- correct increment of:
  - simulation step
  - simulation time

The tester validates that one temporal cycle produces:

- exactly one expected logical advancement in the observed simulation state
- a `step_clock` message with:
  - `delta_step = 1`
  - `delta_time = dt`

For the minimal test world, `dt = 1 ms`.

---

## What this package does not test

This package does not try to validate every possible control-group topology.

In particular:

- it uses a single synthetic control group
- it does not test multiple concurrent groups
- it does not validate more advanced control computations
- it does not validate reset handling

Those concerns belong to other dedicated tests.

---

## Package structure

```text
provant_temporal_loop_tests/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── temporal_loop_test.launch.py
├── src/
│   └── temporal_loop_tester_node.cpp
└── worlds/
    └── temporal_loop_minimal.sdf
```

---

## Components

### 1. Minimal world

File:

```text
worlds/temporal_loop_minimal.sdf
```

This world provides a minimal Gazebo Harmonic environment for the integration test.

It includes:

- physics configuration with a fixed step size
- the `provant_step_manager_system` plugin

This is the Gazebo side responsible for publishing `/provant_simulator/step_clock`.

---

### 2. Temporal loop tester node

File:

```text
src/temporal_loop_tester_node.cpp
```

This node acts as a synthetic control group and as the temporal assertion point for the test.

Its responsibilities are:

- subscribe to `/provant_simulator/step_clock`
- subscribe to `/provant_simulator/step`
- register itself as a control group in `simulation_manager`
- publish `/<group_namespace>/ready`
- request `start` on `simulation_manager`
- verify that the observed temporal state advances correctly

The tester publishes `ready` using:

```text
provant_simulator_interfaces/msg/Empty
```

This must match the message type expected by `simulation_manager`.

---

### 3. Launch test

File:

```text
launch/temporal_loop_test.launch.py
```

This launch file starts the complete temporal loop test environment:

- Gazebo Harmonic with the minimal world
- `provant_step_command_bridge`
- `simulation_manager`
- the temporal loop tester node

The launch test succeeds only if the tester node reports success.

---

## Communication contract used by the test

### Control group ready topic

The tester publishes:

```text
/<group_namespace>/ready
```

Type:

```text
provant_simulator_interfaces/msg/Empty
```

This represents:

> the control group has finished its work and is ready for the next simulation advancement

---

### Simulation step topic

The tester observes:

```text
/provant_simulator/step
```

Type:

```text
std_msgs/msg/Empty
```

Each message means:

> the simulator manager decided to release one discrete simulation step

---

### Step clock topic

The tester observes:

```text
/provant_simulator/step_clock
```

Type:

```text
provant_simulator_interfaces/msg/StepClock
```

This topic carries the observable temporal state after the requested simulation progression.

---

## Validation strategy

The tester performs the following sequence:

1. wait for an initial `step_clock`
2. pause the world and wait for stabilization
3. register a synthetic control group
4. request `start` on `simulation_manager`
5. publish one `ready`
6. observe:
   - one `/provant_simulator/step`
   - one valid next `step_clock`
7. validate:
   - `delta_step == 1`
   - `delta_time == expected_step_dt_ns`

This sequence is repeated for a configurable number of cycles.

---

## Parameters

The tester node supports the following parameters.

### `total_cycles`

Default:

```text
5
```

Number of temporal cycles to validate.

### `request_period_ms`

Default:

```text
300
```

Delay before publishing each new `ready`.

### `compute_delay_ms`

Default:

```text
20
```

Artificial delay used to simulate control computation after receiving a valid `step_clock`.

### `startup_wait_ms`

Default:

```text
1500
```

Initial wait before requesting a pause and starting the test sequence.

### `world_name`

Default:

```text
temporal_loop_test
```

Gazebo world name used to build the world control service path.

### `group_namespace`

Default:

```text
/test_group
```

Namespace used for the synthetic control group.

### `expected_step_dt_ns`

Default:

```text
1000000
```

Expected time increment in nanoseconds for one simulation step.

For the minimal test world:

```text
1 ms = 1,000,000 ns
```

### `time_tolerance_ns`

Default:

```text
0
```

Tolerance allowed when validating the simulation time increment.

---

## Expected successful behavior

A successful run should show logs similar to:

```text
simulation_manager is running. Starting cycles.
Published /test_group/ready for cycle 1/5.
simulation_manager published /provant_simulator/step. count=1
Observed candidate step_clock for cycle 1/5: step=96 delta_step=1 time_ns=95000000 delta_time_ns=1000000
Cycle 1/5 succeeded.
...
Test succeeded.
```

---

## How to run

Build:

```bash
colcon build --packages-select \
  provant_simulator_gz_plugins \
  provant_simulator_step_command_bridge \
  provant_simulator_simulation_manager \
  provant_temporal_loop_tests
```

Source the workspace:

```bash
source install/setup.bash
```

Run the test:

```bash
colcon test --packages-select provant_temporal_loop_tests --event-handlers console_direct+
```

Show the result:

```bash
colcon test-result --verbose
```

You can also launch it directly for debugging:

```bash
ros2 launch provant_temporal_loop_tests temporal_loop_test.launch.py
```

---

## Relationship with other packages

This package is an integration test for:

- `provant_simulator_simulation_manager`
- `provant_simulator_step_command_bridge`
- `provant_step_manager_system`

It does not replace the package-specific unit or smoke tests for those components.

Instead, it validates the complete temporal loop when all of them are used together.
