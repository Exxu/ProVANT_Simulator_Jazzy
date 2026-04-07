# provant_step_manager_system_tests

This package contains a minimal smoke test for the `provant_step_manager_system` plugin running on **ROS 2 Jazzy** with **Gazebo Harmonic**.

Its purpose is to validate the first migration step of ProVANT's temporal semantics from Gazebo Classic to Gazebo Harmonic, focusing only on:

- temporal observation from Gazebo
- reset detection
- reset propagation to ROS 2

It does **not** validate step commands yet. That will be covered later by the `step_command_bridge`.

---

## Goal

The goal of this test package is to verify that the new `provant_step_manager_system` reproduces the expected observable behavior of the legacy `step_manager` in the parts related to:

- publishing simulation step information through `/provant_simulator/step_clock`
- detecting world resets inside Gazebo
- notifying ROS 2 through `/provant_simulator/reset`
- publishing a valid post-reset `step_clock`

This is the first piece required to preserve the temporal contract between:

- `simulation_manager`
- `step_manager`
- Gazebo Harmonic

---

## What is being tested

The test checks four things:

1. **The plugin loads correctly**
   - Gazebo starts with a minimal SDF world
   - `provant_step_manager_system` is loaded as a system plugin

2. **The plugin publishes `/provant_simulator/step_clock`**
   - the test node subscribes to the topic
   - at least one message must be received before reset

3. **The plugin detects world reset and calls `/provant_simulator/reset`**
   - the test node provides the ROS 2 service `/provant_simulator/reset`
   - the plugin must call it after the world reset is triggered

4. **The plugin publishes a post-reset `step_clock`**
   - after reset, a new `step_clock` must be observed
   - the observed step/time must show that the simulation rewound to a lower temporal state

---

## Why this test exists

In Gazebo Classic, the old `step_manager` was responsible for:

- publishing the simulation step clock
- reacting to world reset
- notifying ROS 2 when reset happened

In Gazebo Harmonic, this behavior is now implemented by `provant_step_manager_system`.

Before implementing step-by-step control through `step_command_bridge`, we first need to prove that the system plugin already preserves the **observable temporal semantics** of the simulator.

This test is a minimal proof that:

- the plugin sees simulation progress
- the plugin sees reset events
- the plugin exposes both back to ROS 2

---

## Package structure

```text
provant_step_manager_system_tests/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   └── step_manager_system_smoke.launch.py
├── src/
│   └── test_step_manager_system_node.cpp
└── worlds/
    └── step_manager_system_minimal.sdf
```

---

## Components

### 1. Minimal SDF world

File:

```text
worlds/step_manager_system_minimal.sdf
```

This world contains:

- Gazebo Harmonic physics system
- user commands system
- scene broadcaster system
- the custom plugin `provant_step_manager_system`

Its only purpose is to provide the smallest possible world in which the plugin can run and publish temporal information.

### 2. Test node

File:

```text
src/test_step_manager_system_node.cpp
```

This ROS 2 node performs the verification logic.

It:

- subscribes to `/provant_simulator/step_clock`
- provides the `/provant_simulator/reset` service
- records temporal information before reset
- waits for a new `step_clock` after reset
- exits successfully when the expected behavior is observed

### 3. Launch file

File:

```text
launch/step_manager_system_smoke.launch.py
```

This launch file:

- sets `GZ_SIM_PLUGIN_PATH` so Gazebo can find the custom plugin
- starts Gazebo Harmonic with the minimal world
- starts the test node
- triggers a world reset after a short delay using `gz service`

---

## Test sequence

The smoke test follows this sequence:

1. Launch Gazebo Harmonic with the minimal world
2. Load `provant_step_manager_system`
3. Start the ROS 2 test node
4. Wait until at least one `/provant_simulator/step_clock` is received
5. Trigger a world reset using Gazebo's world control service
6. Wait for the plugin to call `/provant_simulator/reset`
7. Wait for a new `/provant_simulator/step_clock` after reset
8. Check that the post-reset step/time is smaller than the pre-reset values
9. Exit with success

---

## Validation criteria

The test is considered successful only if all the following conditions are met.

### A. `step_clock` is published before reset

The test must receive at least one message on:

```text
/provant_simulator/step_clock
```

This confirms that the plugin is attached to the simulation loop and is publishing temporal observations.

### B. Reset is propagated to ROS 2

The plugin must call:

```text
/provant_simulator/reset
```

This confirms that the plugin correctly reacts to world reset events inside Gazebo.

### C. A new `step_clock` is published after reset

After reset is detected, the plugin must publish another `step_clock`.

This confirms that the plugin preserves temporal observability even after the world is rewound.

### D. The temporal state actually rewound

The test compares the values observed before and after reset.

The post-reset message must indicate a smaller temporal state, such as:

- lower step count
- lower simulation time

For example:

- pre-reset: `step=3093`, `time_ns=3092000000`
- post-reset: `step=2`, `time_ns=2000000`

This proves that the world reset was not only detected, but also reflected in the published temporal state.

---

## Expected successful output

A successful run should contain messages similar to:

```text
StepManagerSystem configured. step_clock_topic='/provant_simulator/step_clock', reset_service='/provant_simulator/reset'
First step_clock received: step=2 time_ns=1000000
World reset detected. Reset notification scheduled.
Reset service called. Captured pre-reset state: step=3093 time_ns=3092000000
Reset notification sent successfully.
Success: post-reset step_clock observed. pre_reset_step=3093 post_reset_step=2 pre_reset_time_ns=3092000000 post_reset_time_ns=2000000
```

The most important line is:

```text
Success: post-reset step_clock observed ...
```

That is the final proof that the smoke test passed.

---

## How to build

From the workspace root:

```bash
colcon build --packages-select provant_simulator_gz_plugins provant_step_manager_system_tests
```

Then source the workspace:

```bash
source install/setup.bash
```

---

## How to run

Run the smoke test with:

```bash
ros2 launch provant_step_manager_system_tests step_manager_system_smoke.launch.py
```

---

## How the reset is triggered

The launch file triggers the world reset using Gazebo transport:

```bash
gz service \
  -s /world/step_manager_test/control \
  --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean \
  --timeout 3000 \
  --req 'reset: {all: true}'
```

This causes Gazebo Harmonic to rewind the simulation, which should then be observed by the plugin.

---

## Interpretation of the result

### Success

If the test node exits cleanly with a success message, then:

- the plugin is loading correctly
- the plugin is publishing temporal data
- the plugin is detecting reset
- the plugin is notifying ROS 2
- the plugin is publishing a valid post-reset temporal observation

### Failure

The test should be considered failed if any of these happens:

- no `step_clock` is received
- `/provant_simulator/reset` is never called
- no post-reset `step_clock` is observed
- the test times out

---

## Scope of this test

This test validates only the **observation/reset** responsibilities of the new plugin.

It does **not** validate:

- step command handling
- single-step execution semantics
- interaction with `simulation_manager`
- interaction with `step_command_bridge`

Those behaviors belong to the next migration steps.

---

## Current architectural meaning

At this stage, the migration is split as follows:

- `step_manager_system`
  - observes simulation time
  - publishes `/provant_simulator/step_clock`
  - detects world reset
  - calls `/provant_simulator/reset`

- `step_command_bridge`
  - not implemented in this test yet
  - will be responsible for requesting discrete simulation steps in Gazebo Harmonic

This package only validates the first half of that contract.