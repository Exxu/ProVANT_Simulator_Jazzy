# provant_controller_zoh_loop_test

This package contains the integration test for the interaction between:

- `provant_simulator_simulation_manager`
- `provant_simulator_control_group_manager`
- `provant_simulator_step_command_bridge`
- `provant_step_manager_system`
- `provant::Controller`
- `provant_simulator_zoh`
- Gazebo Harmonic

Its purpose is to verify that the migrated temporal loop works correctly when a real controller and a real ZOH are inserted into the loop.

This package goes beyond the bridge-only and paused-loop tests because it validates the complete temporal behavior seen by a control group.

---

## Goal

The goal of this package is to validate that the **global temporal loop** and the **group-level control loop** work together correctly.

More specifically, it verifies that:

- the world can be brought into a paused state
- the pause condition can be confirmed by the absence of new `/provant_simulator/step_clock` messages
- `simulation_manager` can be switched to `running` without the world advancing on its own
- one bootstrap message on `/provant_simulator/step` starts the closed temporal loop
- `control_group_manager` triggers control updates only on the configured control period
- `provant_simulator_zoh` holds the last control value between control updates
- the actuator output follows the expected controller/ZOH pattern when observed against the **global** simulation clock

This confirms that the migrated controller + ZOH temporal loop is functioning.

---

## Architectural role

This package tests the integration of the migrated temporal components and the control path on top of them.

It covers the following roles:

- `provant_simulator_simulation_manager`
  - waits for all registered groups to become ready
  - publishes `/provant_simulator/step`

- `provant_simulator_step_command_bridge`
  - receives `/provant_simulator/step`
  - sends a Gazebo world control request for one discrete step

- `provant_step_manager_system`
  - observes the resulting simulation progression
  - publishes `/provant_simulator/step_clock`

- `provant_simulator_control_group_manager`
  - consumes the global step clock
  - decides when to publish local control ticks
  - triggers ZOH between control updates
  - publishes `ready` back to `simulation_manager`

- `provant::Controller`
  - computes `control_inputs` when the local control tick is published

- `provant_simulator_zoh`
  - stores the last `control_inputs`
  - republishes that value on actuator topics when triggered between control updates

This package tests the closed loop below:

```text
/provant_simulator/step
    -> provant_step_command_bridge
        -> Gazebo world control
            -> provant_step_manager_system
                -> /provant_simulator/step_clock
                    -> provant_simulator_control_group_manager
                        -> /test_group/step_clock       (control only)
                        -> /test_group/zoh_trigger      (intermediate steps)
                            -> controller / ZOH / actuator
                                -> /test_group/ready
                                    -> simulation_manager
                                        -> /provant_simulator/step
```

---

## Why this test exists

In Gazebo Classic, the original temporal behavior depended on a tighter coupling between stepping and observation.

In Gazebo Harmonic, the temporal architecture is explicitly split:

- global step command path
- global step observation path
- group scheduler path
- controller/ZOH execution path

Because of that split, it is not enough to test only:

- `provant_step_command_bridge`
- `provant_step_manager_system`
- `provant_simulator_control_group_manager`

in isolation.

This package exists to verify that the migrated temporal architecture still preserves the expected ProVANT semantics once a real controller and a real ZOH are inserted into the loop.

---

## Package structure

```text
provant_controller_zoh_loop_test/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   └── provant_controller_zoh_loop_test.launch.py
├── src/
│   ├── controller_input_source_node.cpp
│   ├── test_controller_node.cpp
│   └── provant_controller_zoh_loop_tester_node.cpp
└── worlds/
    └── controlloop_minimal.sdf
```

---

## Components

### 1. Minimal world

File:

```text
worlds/controlloop_minimal.sdf
```

This world is intentionally minimal.

It contains the Gazebo systems required to support the temporal loop, including `provant_step_manager_system`.

Its role is to provide a controlled simulation world where the full controller + ZOH temporal loop can be observed without unrelated simulator complexity.

---

### 2. Controller input source

File:

```text
src/controller_input_source_node.cpp
```

This ROS 2 node provides deterministic inputs to the controller.

It subscribes to the group local `step_clock` and publishes:

- `references`
- `state_vector`

For each control step it uses a deterministic rule:

- `reference = 2 * step`
- `state = step`

so the expected controller output is:

- `control = reference - state = step`

This makes the expected actuator behavior easy to validate.

---

### 3. Test controller

File:

```text
src/test_controller_node.cpp
```

This node instantiates a real `provant::Controller` using a deterministic controller implementation.

It consumes:

- `references`
- `state_vector`

and publishes:

- `control_inputs`

Its control law is intentionally simple:

```text
u = reference - state
```

The purpose is not to test control performance, but to make controller output fully predictable for the integration test.

---

### 4. ZOH node

Launched component:

```text
provant_simulator_zoh/control_zoh_node
```

This is the real ZOH used by the simulator.

It receives:

- `control_inputs`
- `zoh_trigger`

and publishes the held actuator value.

Its role in this package is to prove that, between controller updates, the actuator output remains equal to the last computed control value.

---

### 5. Tester node

File:

```text
src/provant_controller_zoh_loop_tester_node.cpp
```

This ROS 2 node performs the integration validation logic.

It:

- subscribes to `/provant_simulator/step_clock`
- subscribes to the actuator topic
- observes `/provant_simulator/simulation_state`
- observes `/provant_simulator/step`
- requests an initial world pause through Gazebo world control
- verifies the pause by waiting for a quiet period with no new global `step_clock`
- sends `start` to `simulation_manager`
- verifies that no new global `step_clock` arrives before bootstrap
- publishes one bootstrap message on `/provant_simulator/step`
- waits for the first post-bootstrap global step
- validates the controller + ZOH behavior against the **global** timeline

This package intentionally uses the **global** `step_clock` as the test timeline.

It does **not** use the local `/<group>/step_clock` as a continuous clock, because that topic is only published on control-update steps.

---

### 6. Launch test

File:

```text
launch/provant_controller_zoh_loop_test.launch.py
```

This launch file starts:

- Gazebo Harmonic server
- `provant_simulator_step_command_bridge`
- `provant_simulator_simulation_manager`
- `provant_simulator_control_group_manager`
- `controller_input_source_node`
- `test_controller_node`
- `provant_simulator_zoh/control_zoh_node`
- `provant_controller_zoh_loop_tester_node`

It also registers the package as a `launch_testing` integration test.

---

## What is being tested

The package verifies all of the following:

1. `provant_step_manager_system` loads correctly
2. `provant_step_command_bridge` starts correctly
3. `simulation_manager` starts correctly
4. `control_group_manager` starts correctly and registers its group
5. the world control service is reachable
6. the world can be paused
7. the pause can be confirmed by the absence of new global `step_clock` messages
8. `simulation_manager` can enter `running` without causing uncontrolled world progression
9. one bootstrap `/provant_simulator/step` starts the temporal loop
10. the controller is executed on the configured control period
11. the ZOH holds the last controller output on intermediate steps
12. the actuator output matches the expected controller/ZOH pattern

---

## Communication contract under test

### Global step request

The test observes and bootstraps:

```text
/provant_simulator/step
```

Type:

```text
std_msgs/msg/Empty
```

A bootstrap message on this topic means:

> request one discrete simulation step

After the loop is running, `simulation_manager` continues publishing it whenever all groups are ready.

### Global step observation

The tester subscribes to:

```text
/provant_simulator/step_clock
```

Type:

```text
provant_simulator_interfaces/msg/StepClock
```

This topic is the canonical timeline for the test.

Each received message indicates one observed world step/time state.

### Group-level control trigger

The controller input source and the controller depend on:

```text
/test_group/step_clock
```

Type:

```text
provant_simulator_interfaces/msg/StepClock
```

This topic is **not** a continuous clock.

It is only published by `control_group_manager` when it is time to compute control.

### ZOH trigger

The ZOH depends on:

```text
/test_group/zoh_trigger
```

Type:

```text
provant_simulator_interfaces/msg/StepClock
```

This topic is published by `control_group_manager` on intermediate steps where control is not recomputed.

### Actuator output

The tester validates:

```text
/test_group/actuator_1
```

Type:

```text
provant_simulator_interfaces/msg/Actuator
```

This topic must reflect either:

- a new controller output on control steps
- the held output from ZOH on intermediate steps

---

## How the test works

The test follows this sequence:

1. Start Gazebo Harmonic with the minimal world
2. Load `provant_step_manager_system`
3. Start `provant_step_command_bridge`
4. Start `simulation_manager`
5. Start `control_group_manager`
6. Start the deterministic controller input source
7. Start the deterministic controller
8. Start the real ZOH node
9. Start the tester node
10. Wait until at least one global `step_clock` is received
11. Send `pause: true` to the world control service
12. Wait until no new global `step_clock` is observed for a quiet period
13. Wait for all components to settle and connect
14. Send `start` to `simulation_manager`
15. Verify that no new global `step_clock` arrives after `start`
16. Publish one bootstrap message to `/provant_simulator/step`
17. Observe the first post-bootstrap global `step_clock`
18. Use the global step sequence as the test timeline
19. Validate that controller updates happen only every `control_period_steps`
20. Validate that ZOH holds the last control value between those updates
21. Finish successfully if all observed actuator messages match the expected pattern

---

## Why the test pauses the world first

For the simulator to preserve ProVANT temporal semantics, the world must first be brought under controlled discrete stepping.

If the world is not paused, then controller and ZOH behavior may be mixed with uncontrolled free-running simulation progression, making temporal validation ambiguous.

That is why the test first requests:

```text
pause: true
```

and then waits for a quiet period with no new global `step_clock` messages.

Only after that does it continue.

---

## Why pause is verified by silence on the global `step_clock`

Once the world is paused, `provant_step_manager_system` stops publishing new `step_clock` messages because the world is no longer advancing.

Because of that, the test does **not** validate pause by comparing two equal `step_clock` messages.

Instead, it validates pause by observing that:

- new global `step_clock` messages stop arriving
- that quiet period persists for a configured duration

This is the correct interpretation of pause in the migrated Harmonic architecture.

---

## Why `start` is not enough

The test explicitly verifies that switching `simulation_manager` to `running` is not enough to make the world advance by itself.

This matters because the migrated temporal architecture is barrier-based.

After `start`, the world should still remain paused until the closed loop is seeded.

That is why the tester waits for another quiet period after `start`, and only then sends the bootstrap step.

---

## Why the bootstrap uses `/provant_simulator/step`

This package seeds the loop with exactly one message on:

```text
/provant_simulator/step
```

It does **not** bootstrap using `/<group>/ready`.

That is intentional.

The purpose is to prove that the global temporal loop starts only when one discrete global step is requested, and not because the tester manually closes the group barrier.

This makes the test closer to the actual migrated temporal architecture.

---

## Why the global clock is used as the test timeline

The tester does **not** use the local `/<group>/step_clock` as the main timeline.

That is intentional.

The local group `step_clock` is only published on control-update steps.
On intermediate steps, `control_group_manager` publishes only `zoh_trigger`.

Because of that, the correct continuous timeline for the test is:

```text
/provant_simulator/step_clock
```

The tester interprets control-step spacing relative to the global step sequence.

---

## Validation criteria

The test is considered successful only if all the following conditions are met.

### A. Initial global `step_clock` is received

The tester must observe at least one valid `/provant_simulator/step_clock` message.

This confirms that `provant_step_manager_system` is loaded and publishing.

### B. The world accepts the pause request

The tester sends a world control request with:

```text
pause: true
```

This confirms that the Gazebo control service is reachable and the world can enter a paused state.

### C. Global step progression stops after pause

After the pause request, the tester must observe a quiet period with no new global `step_clock` messages.

This confirms that the world is no longer advancing freely.

### D. `simulation_manager` enters `running`

The tester must observe the `running` state on:

```text
/provant_simulator/simulation_state
```

This confirms that the global temporal coordinator has transitioned to the execution state.

### E. No uncontrolled progression occurs after `start`

After `simulation_manager` enters `running`, the tester must still observe no new global `step_clock` messages until the bootstrap step is sent.

This confirms that the loop does not start by itself.

### F. The bootstrap step produces the first observed progression

After publishing one bootstrap `/provant_simulator/step`, the tester must observe the first new global `step_clock`.

This confirms that one discrete step request is enough to seed the temporal loop.

### G. Controller updates happen only on control-period steps

The tester derives control-step positions from the global step timeline and the configured `control_period_steps`.

At those positions, the actuator must reflect a newly computed controller output.

### H. ZOH holds the control value on intermediate steps

On all intermediate global steps between control updates, the actuator must keep the same previously computed value.

This is the core ZOH validation of the package.

### I. The actuator values match the deterministic controller law

Since the controller input source publishes:

```text
reference = 2 * step
state = step
```

and the controller computes:

```text
u = reference - state
```

then the expected actuator value on each control step is:

```text
u = step
```

The tester validates this relationship directly.

---

## Expected successful output

A successful run should contain messages similar to:

```text
Controller + ZOH tester started. world='temporal_loop_test' group='/test_group' actuator_topic='actuator_1' required_actuator_msgs=7 control_period_steps=3 expected_step_dt_ns=1000000
Initial global step_clock received: step=2 time_ns=1000000
Pause requested. Waiting for 600 ms without new global step_clock messages. current_step=36 current_time_ns=35000000
World appears paused. No new global step_clock for 610 ms. Waiting 1500 ms for control_group/controller/ZOH registration and connections.
Requested simulation_manager state 'start'.
simulation_manager state='running'
simulation_manager is running. Verifying that no new global step_clock arrives before bootstrap for 1000 ms.
No new global step_clock arrived for 1004 ms after start. Sending one bootstrap /provant_simulator/step.
First post-bootstrap global step_clock observed: step=89 delta=1 delta_time_ns=1000000
Using global step 89 as first control step baseline.
Validated actuator msg 1/7 at global step=89 kind=CONTROL expected=89.000000 actual=89.000000
Validated actuator msg 2/7 at global step=90 kind=ZOH expected=89.000000 actual=89.000000
Validated actuator msg 3/7 at global step=91 kind=ZOH expected=89.000000 actual=89.000000
Validated actuator msg 4/7 at global step=92 kind=CONTROL expected=92.000000 actual=92.000000
Validated actuator msg 5/7 at global step=93 kind=ZOH expected=92.000000 actual=92.000000
Validated actuator msg 6/7 at global step=94 kind=ZOH expected=92.000000 actual=92.000000
Validated actuator msg 7/7 at global step=95 kind=CONTROL expected=95.000000 actual=95.000000
provant_controller_zoh_loop_test succeeded.
```

The most important line is:

```text
provant_controller_zoh_loop_test succeeded.
```

That indicates the temporal integration behaved as expected.

---

## How to build

From the workspace root:

```bash
colcon build --packages-select \
  provant_simulator_gz_plugins \
  provant_simulator_step_command_bridge \
  provant_simulator_simulation_manager \
  provant_simulator_control_group_manager \
  provant_simulator_zoh \
  provant_simulator_controller \
  provant_controller_zoh_loop_test
```

Then source the workspace:

```bash
source install/setup.bash
```

---

## How to run as automated test

Run the test with:

```bash
colcon test --packages-select provant_controller_zoh_loop_test
```

Then inspect the result:

```bash
colcon test-result --verbose
```

---

## How to run with direct console output

If you want to see the integration log directly in the terminal, run:

```bash
colcon test --packages-select provant_controller_zoh_loop_test --event-handlers console_direct+
```

This is useful when you want to observe the pause verification, bootstrap step, global step progression, and actuator validation in real time.

---

## What this package does not test

This package does **not** test:

- multi-group synchronization across several control groups
- disturbance generation behavior
- estimator behavior
- controller quality or closed-loop stability
- drone dynamics fidelity
- reset propagation semantics in depth
- complex controller topologies

Those belong to other integration or system-level tests.

---

## Relationship with the migrated temporal architecture

This package sits above the lower-level bridge tests.

It assumes that:

- one `/provant_simulator/step` request can advance the world by one step
- `provant_step_manager_system` can report that step through `/provant_simulator/step_clock`

and validates the next integration layer on top of that:

- `simulation_manager`
- `control_group_manager`
- controller execution
- ZOH hold semantics

In other words, it is the first integration proof that the migrated temporal architecture still behaves correctly once real control execution is inserted into the loop.

---

## Expected startup condition

This test assumes that the world must be paused before the global temporal loop is seeded.

This matches the simulator requirement that no free-running world progression should occur before:

- the group is registered
- the controller is ready
- the ZOH is ready
- the test explicitly starts the loop

---

## Failure conditions

The test should be considered failed if any of the following happens:

- no global `step_clock` is received
- the world cannot be paused
- the quiet-period pause verification never succeeds
- `simulation_manager` never enters `running`
- a new global `step_clock` appears before the bootstrap step
- the bootstrap step does not produce a new observed global step
- the actuator output does not match the expected control-step value
- the actuator output is not held correctly by the ZOH on intermediate steps
- the tester times out

These indicate that the migrated controller + ZOH temporal loop is not behaving as required.

---

## Scope of migration

This package is part of the migration from:

- ROS 2 Iron + Gazebo Classic

to:

- ROS 2 Jazzy + Gazebo Harmonic

Its specific contribution is to validate that the new split temporal architecture, together with `simulation_manager`, `control_group_manager`, a real `provant::Controller`, and the real ZOH, can jointly reproduce the temporal behavior expected by ProVANT.

---

## Summary

`provant_controller_zoh_loop_test` is the integration test package that verifies the migrated temporal loop once controller execution and ZOH hold semantics are included.

In practical terms, it confirms that:

- the world can be paused and that pause can be confirmed correctly
- `start` alone does not advance the world
- one bootstrap global step seeds the closed temporal loop
- controller updates occur on the configured period
- ZOH holds the last control value between updates
- the actuator output matches the expected deterministic pattern

This package is the controller/ZOH integration proof that the migrated Harmonic-based temporal architecture is behaving correctly.
