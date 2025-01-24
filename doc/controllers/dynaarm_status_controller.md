# DynaArm Status Controller

Controller that provides additional status information about the configured DynaArm. 
The published [ArmState](https://github.com/Duatic/dynaarm_driver/blob/main/dynaarm_msgs/msg/ArmState.msg) consists of an array of information about each configured [actuator](https://github.com/Duatic/dynaarm_driver/blob/main/dynaarm_msgs/msg/DriveState.msg)

## Parameters:

### Definition:
```{literalinclude} ../../dynaarm_controllers/src/dynaarm_status_controller_parameters.yaml
```

__joints__ | [Required]:

List of managed joints by the controller. Always needs to be a list of all joints of an arm in the order specified in the urdf

__arm_name__ | [Required]:

Name of the arm as specified in the ros2control part of the urdf. Currently this is `${tf_prefix}DynaarmSystem`


### Example:

A full example can be found in the [dynaarm_demo](https://github.com/Duatic/dynaarm_demo/blob/main/dynaarm_examples/config/controllers.yaml) repository.

```yaml
dynaarm_status_controller:
  ros__parameters:
    joints:
      - shoulder_rotation
      - shoulder_flexion
      - elbow_flexion
      - forearm_rotation
      - wrist_flexion
      - wrist_rotation
    arm_name: DynaarmSystem
```

## ROS Interfacing

### Topics:

__~/state__ | [Publisher]:\
type: [<dynaarm_msgs/msg/ArmState>](https://github.com/Duatic/dynaarm_driver/blob/main/dynaarm_msgs/msg/ArmState.msg)
Publishes the ArmState message with the configured controller update rate.

## References

* [Source](https://github.com/Duatic/dynaarm_driver/blob/main/dynaarm_controllers/include/dynaarm_controllers/dynaarm_status_controller.hpp)

