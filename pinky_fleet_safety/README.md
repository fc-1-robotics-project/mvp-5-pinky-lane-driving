# Pinky fleet velocity safety gate

This package makes the robot-local gate the only publisher allowed to command
the motor driver directly.

```text
Nav2 controller / behavior server
              -> cmd_vel_nav
              -> velocity_smoother
              -> cmd_vel_candidate
              -> fleet_velocity_gate
              -> cmd_vel
              -> pinky_bringup motor driver
```

The gate starts fail-closed and continuously publishes zero velocity until it
has both a fresh `MODE_RUN` permit and a fresh candidate velocity command.
`MODE_HOLD`, `MODE_ESTOP`, an expired permit, or an expired candidate command
all force zero velocity. The motor driver has its own 0.3 second watchdog as a
second layer in case the gate process exits.

## Files to transfer to each robot

Copy the new `pinky_fleet_safety` package. Apply these repository changes too:

1. Add `FleetPermit.msg` and `RobotHeartbeat.msg` to `pinky_interfaces/msg/`
   and register both files in `pinky_interfaces/CMakeLists.txt`.
2. Change both velocity-smoother output remaps in
   `pinky_navigation/launch/navigation_launch.xml` from `cmd_vel` to
   `cmd_vel_candidate`. Remap `behavior_server` from `cmd_vel` to
   `cmd_vel_nav` in both composable and standalone sections.
3. Start `pinky_fleet_safety/velocity_gate` from
   `pinky_bringup/launch/bringup_robot.launch.xml`, passing the robot ID.
4. Add the `command_timeout_sec` parameter and timeout enforcement from
   `pinky_bringup/pinky_bringup/bringup.py` and `config/pinky_params.yaml`.
5. Make every additional local or bridged velocity publisher publish to
   `cmd_vel_candidate`, never directly to `cmd_vel`. The included
   `pinky_control/just_move.py`, `vision_control.py`, and
   `config/vision_control.yaml` have been updated accordingly. The control-PC
   `domain_bridge` destination for manual velocity commands must also change
   from robot `/cmd_vel` to robot `/cmd_vel_candidate`.

The control PC also needs the exact same `pinky_interfaces` message definitions
to publish permits and decode heartbeat messages.

## Build and run

```bash
cd ~/pinky
colcon build --packages-select \
  pinky_interfaces pinky_fleet_safety pinky_bringup pinky_navigation
source install/setup.bash
```

Robot 1:

```bash
ROS_DOMAIN_ID=21 ros2 launch pinky_bringup bringup_robot.launch.xml \
  robot_id:=robot1
ROS_DOMAIN_ID=21 ros2 launch pinky_navigation bringup_launch.xml \
  map:=my_map.yaml
```

Robot 2 uses `ROS_DOMAIN_ID=19` and `robot_id:=robot2`.

For a bench test with the wheels lifted, send a short-lived RUN permit at a
rate faster than its TTL. Repeating the same sequence refreshes the permit;
lower sequence numbers from the same controller session are rejected.

```bash
ros2 topic pub -r 10 /fleet/permit pinky_interfaces/msg/FleetPermit \
  "{robot_id: robot1, controller_id: test-session, sequence: 1, mode: 1, \
  ttl: {sec: 0, nanosec: 500000000}, lease_id: '', allowed_zone_ids: []}"
```

Stopping this publisher must change `/fleet/heartbeat.status` to
`PERMIT_TIMEOUT` and force `/cmd_vel` to zero within 0.5 seconds. Use
`mode: 0` for HOLD and `mode: 2` for E-STOP.