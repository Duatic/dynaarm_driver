# Gravity Compensation Controller

Controller that calculates and commands feed forward torques for the current configuration and velocities.\
It is recommended to always run this controller as it vastly improves the control performance.

## Parameters

### Definition:
```{literalinclude} ../../dynaarm_controllers/src/gravity_compensation_controller_parameters.yaml
```

__joints__ | [Required]:\
List of managed joints by the controller. Always needs to be a list of all joints of an arm in the order specified in the urdf

__arm_name__ | [Required]:\
Name of the arm as specified in the ros2control part of the urdf. Currently this is `${tf_prefix}DynaarmSystem`

### Example:

A full example can be found in the [dynaarm_demo](https://github.com/Duatic/dynaarm_demo/blob/main/dynaarm_examples/config/controllers.yaml) repository.

```yaml
gravity_compensation_controller:
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

None

## Additional Information

The controller uses internally the [pinnocchio::rnea](https://github.com/stack-of-tasks/pinocchio/blob/master/include/pinocchio/algorithm/rnea.hpp) function in order to calculate the inverse dynamics.\
As written above it is recommend to always run this controller. The only reason not to run this controller is in you want to command custom torques to the arm.\ The controller does not allow controller chaining at the moment.


## References

* [Source](https://github.com/Duatic/dynaarm_driver/blob/main/dynaarm_controllers/include/dynaarm_controllers/gravity_compensation_controller.hpp)
