# provant_step_command_bridge_tests

This package contains the integration tests for the interaction between:

- `provant_step_command_bridge`
- `provant_step_manager_system`
- Gazebo Harmonic

Its purpose is to verify that discrete step requests issued from ROS 2 actually produce observable single-step advances in the simulation.

This test package is independent from the smoke test created only for `provant_step_manager_system`.

---

## Goal

The goal of this package is to validate that the **command side** and the **observation side** of the migrated temporal architecture work together correctly.

More specifically, it verifies that:

- the world can be brought into a paused state
- each message published on `/provant_simulator/step` produces one discrete world step
- `provant_step_manager_system` reports that advance through `/provant_simulator/step_clock`

This confirms that the temporal loop between step requests and step observations is functioning.

---

## Architectural role

This package tests the integration of two migrated components:

- `provant_step_command_bridge`
  - receives ROS 2 step requests
  - sends a Gazebo world control request

- `provant_step_manager_system`
  - observes the resulting simulation progression
  - publishes `/provant_simulator/step_clock`

This package does **not** test the full simulator loop with `simulation_manager`.  
It focuses only on the closed loop between:

```text
/provant_simulator/step
    -> provant_step_command_bridge
        -> Gazebo world control
            -> provant_step_manager_system
                -> /provant_simulator/step_clock
```

---

## Why this test exists

In Gazebo Classic, the old `step_manager` both triggered and observed simulation steps.

In Gazebo Harmonic, those responsibilities are split:

- step command path
- step observation path

Because of that split, it is not enough to test each component in isolation.

This package exists to verify that both components work correctly **together**.

---

## Package structure

```text
provant_step_command_bridge_tests/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   └── step_command_bridge_test.launch.py
├── src/
│   └── step_command_bridge_tester_node.cpp
└── worlds/
    └── step_command_bridge_minimal.sdf
```

---

## Components

### 1. Minimal world

File:

```text
worlds/step_command_bridge_minimal.sdf
```

This world is intentionally minimal.

It contains:

- physics
- user commands system
- scene broadcaster system
- `provant_step_manager_system`

Its role is to provide a controlled simulation world where step requests can be observed without unrelated simulator complexity.

---

### 2. Tester node

File:

```text
src/step_command_bridge_tester_node.cpp
```

This ROS 2 node performs the integration validation logic.

It:

- subscribes to `/provant_simulator/step_clock`
- publishes `/provant_simulator/step`
- exposes `/provant_simulator/reset`
- requests an initial world pause
- verifies that the step clock stabilizes
- issues discrete step requests
- verifies that each request produces exactly one observable step

---

### 3. Launch test

File:

```text
launch/step_command_bridge_test.launch.py
```

This launch file starts:

- Gazebo Harmonic server
- `provant_step_command_bridge`
- the tester node

It also registers the package as a `launch_testing` integration test.

---

## What is being tested

The package verifies all of the following:

1. `provant_step_manager_system` loads correctly
2. `provant_step_command_bridge` starts correctly
3. the world control service is reachable
4. the world can be paused
5. the world step stabilizes after pause
6. each step request causes exactly one observed increment in `step_clock`

---

## Communication contract under test

### Step request

The tester publishes:

```text
/provant_simulator/step
```

Type:

```text
std_msgs/msg/Empty
```

Each message means:

> request one discrete simulation step

### Step observation

The tester subscribes to:

```text
/provant_simulator/step_clock
```

Type:

```text
provant_simulator_interfaces/msg/StepClock
```

Each received message indicates the observable simulation step/time state.

---

## How the test works

The test follows this sequence:

1. Start Gazebo Harmonic with the minimal world
2. Load `provant_step_manager_system`
3. Start `provant_step_command_bridge`
4. Start the tester node
5. Wait until at least one `step_clock` is received
6. Send `pause: true` to the world control service
7. Wait until the observed step stabilizes
8. Publish one message to `/provant_simulator/step`
9. Observe the next `step_clock`
10. Verify that the step increment is exactly `+1`
11. Repeat several times
12. Finish successfully if all step requests behave correctly

---

## Why the test pauses the world first

For the simulator to preserve ProVANT temporal semantics, the world must be under **discrete step control**.

If the world is running freely, then a step request may be mixed with uncontrolled simulation progression, making the temporal behavior ambiguous.

That is why the test first requests:

```text
pause: true
```

and then waits for the observed `step_clock` to stabilize.

Only after that does it start the step-by-step validation.

---

## Why step stabilization is used

The tester does not rely only on a fixed time delay.

Instead, it uses the observed `step_clock` to determine whether the world appears stable.

This is more robust than using a raw delay because it bases the decision on actual simulator state rather than only on wall-clock timing.

---

## Validation criteria

The test is considered successful only if all the following conditions are met.

### A. Initial `step_clock` is received

The tester must observe at least one valid `step_clock` message.

This confirms that `provant_step_manager_system` is loaded and publishing.

### B. The world accepts the pause request

The tester sends a world control request with:

```text
pause: true
```

This confirms that the control service is reachable and the world can enter a controlled state.

### C. Step progression stabilizes

After the pause request, the observed step must stop changing continuously.

This confirms that the world is no longer advancing freely.

### D. Each step request produces exactly one observed increment

For each message sent to:

```text
/provant_simulator/step
```

the next observed progression must be exactly:

```text
delta = 1
```

This is the core validation of the package.

---

## Expected successful output

A successful run should contain messages similar to:

```text
Tester started. total_requests=5 request_period_ms=300 startup_wait_ms=1500 world_name='step_command_bridge_test'
Initial step_clock received: step=2
Pause requested. Waiting for step stabilization from step=32.
World appears paused. Starting step-by-step validation from step=93.
Published step request 1/5 with base step=93.
Step request 1/5 succeeded. Observed delta=1, base=93 new_step=94
Published step request 2/5 with base step=94.
Step request 2/5 succeeded. Observed delta=1, base=94 new_step=95
Published step request 3/5 with base step=95.
Step request 3/5 succeeded. Observed delta=1, base=95 new_step=96
Published step request 4/5 with base step=96.
Step request 4/5 succeeded. Observed delta=1, base=96 new_step=97
Published step request 5/5 with base step=97.
Step request 5/5 succeeded. Observed delta=1, base=97 new_step=98
Test succeeded.
```

The most important line is:

```text
Test succeeded.
```

That indicates the temporal integration behaved as expected.

---

## How to build

From the workspace root:

```bash
colcon build --packages-select \
  provant_simulator_gz_plugins \
  provant_simulator_step_command_bridge \
  provant_step_command_bridge_tests
```

Then source the workspace:

```bash
source install/setup.bash
```

---

## How to run as automated test

Run the test with:

```bash
colcon test --packages-select provant_step_command_bridge_tests
```

Then inspect the result:

```bash
colcon test-result --verbose
```

---

## How to run with direct console output

If you want to see the integration log directly in the terminal, run:

```bash
colcon test --packages-select provant_step_command_bridge_tests --event-handlers console_direct+
```

This is useful when you want to see the progression of steps and validation messages in real time.

---

## What this package does not test

This package does **not** test:

- the full simulator loop including `simulation_manager`
- group registration semantics
- synchronization across multiple controllers
- reset propagation semantics in depth
- drone dynamics or controller behavior

Those belong to later integration steps.

---

## Relationship with `simulation_manager`

This package prepares the ground for integrating `simulation_manager`, but does not include it yet.

Its role is to validate that the lower-level temporal loop already works:

- ROS 2 step request
- Gazebo step execution
- ROS 2 clock observation

Once that is validated, `simulation_manager` can be connected on top of this mechanism.

---

## Expected startup condition

This test assumes that the world must be brought under paused/discrete control before temporal validation begins.

This matches the simulator requirement for aerial vehicles, where starting unpaused can cause bodies to move before the synchronization loop is ready.

---

## Failure conditions

The test should be considered failed if any of the following happens:

- no `step_clock` is received
- the world cannot be paused
- step stabilization never occurs
- a step request does not produce a new step
- a step request produces a step increment different from `+1`
- the tester times out

These indicate that the temporal command/observation loop is not behaving as required.

---

## Scope of migration

This package is part of the migration from:

- ROS 2 Iron + Gazebo Classic

to:

- ROS 2 Jazzy + Gazebo Harmonic

Its specific contribution is to validate that the new split architecture:

- `provant_step_command_bridge`
- `provant_step_manager_system`

can jointly reproduce the step-by-step temporal behavior expected by ProVANT.

---

## Summary

`provant_step_command_bridge_tests` is the integration test package that verifies the interaction between the command-side and observation-side temporal components of the migrated simulator.

In practical terms, it confirms that:

- the world can be brought into paused control
- each ROS 2 step request produces exactly one observed world step
- the new Harmonic-based temporal loop is functioning correctly

This package is the bridge-level integration proof that the migrated temporal architecture is behaving correctly before integrating `simulation_manager`.
