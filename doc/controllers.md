# Controllers

This page contains a list of supported and shipped (custom) controllers. \
Please refer to the [demo](https://github.com/Duatic/dynaarm_demo) repository for some examples on how to set them up.

## Supported controllers

In general all "simple" default ros2_control controllers are supported: [Overview](https://control.ros.org/rolling/doc/ros2_controllers/doc/controllers_index.html#controllers-for-manipulators-and-other-robots) \

In addition we also use for testing the [Cartesian Motion Controller](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers/tree/ros2/cartesian_motion_controller) provided by the FZI.

## Shipped (custom) controllers

In order to fully utilize the DynaArm with ROS2 we ship some additional controllers:

| Name |    Description|
| ---  |  ----         |
| [DynaArmStatusController](./controllers/dynaarm_status_controller.md) | Provide additional status information about the arm that is not provided by any of the default [broadcasters](https://control.ros.org/rolling/doc/ros2_controllers/doc/controllers_index.html#controllers-for-manipulators-and-other-robots) |
| [GravityCompensationController](./controllers/gravity_compensation_controller.md) | Commands feed forward torque commands that compensation the gravity influence based on the current configuration and velocities |
| [FreeDriveController](./controllers/freedrive_controller.md) | Allow to freely move the arm around by hand - needs to be run in combination with the [GravityCompensationController](./controllers/gravity_compensation_controller.md) |
| [FreezeController](./controllers/freeze_controller.md) | Puts the hardware into freeze mode during activation |
| [DynaArmPidTuner](./controllers/dynaarm_pid_tuner.md) | Allows to tune / update the PID parameters in the live system |


```{toctree}
:hidden:
controllers/dynaarm_status_controller.md
controllers/gravity_compensation_controller.md
controllers/freedrive_controller.md
controllers/freeze_controller.md
controllers/dynaarm_pid_tuner.md
```
