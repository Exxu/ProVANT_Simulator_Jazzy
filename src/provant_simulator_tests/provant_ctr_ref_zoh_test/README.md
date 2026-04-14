# provant_ctr_ref_zoh_test

Integration test for the migrated temporal loop using:

- `provant_simulator_simulation_manager`
- `provant_simulator_step_command_bridge`
- `provant_step_manager_system`
- `provant_simulator_control_group_manager`
- `provant_simulator_reference_generator` (`ref_gen_node`, mathematical trajectory)
- `provant::Controller` with a deterministic strategy
- `provant_simulator_zoh`
- Gazebo Harmonic

## Goal

This test extends `provant_controller_zoh_loop_test` by inserting the **real mathematical reference generator** into the loop.

The test validates that:

- the world can be paused and remains paused until bootstrapped
- `simulation_manager` only starts advancing after an explicit bootstrap step
- `control_group_manager` publishes local control ticks every `control_period_steps`
- `ref_gen_node` publishes the expected analytical reference vector for each local control tick
- a fake state source publishes a zero state vector on each local control tick
- the controller output equals the first reference component (`sin(t / 2)`) because `state[0] = 0`
- `control_zoh` holds that value between control ticks

## Closed loop under test

```text
/provant_simulator/step
  -> provant_step_command_bridge
    -> Gazebo world control (multi_step=1)
      -> provant_step_manager_system
        -> /provant_simulator/step_clock
          -> /test_group/control_group_manager
            -> /test_group/step_clock        (control ticks)
              -> /test_group/reference_generator
                -> /test_group/references
              -> /test_group/state_vector_source
                -> /test_group/state_vector
                  -> /test_group/controller
                    -> /test_group/control_inputs
                      -> /test_group/control_zoh
                        -> /test_group/actuator_1
            -> /test_group/zoh_trigger       (intermediate ticks)
              -> /test_group/control_zoh
                -> /test_group/actuator_1
            -> /test_group/ready
              -> /provant_simulator/simulation_manager
```
