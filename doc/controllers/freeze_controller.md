# Freeze Controller

Controller that enables the hardware level freeze mode of the actuators. 
The freeze mode is enabled upon activation of the controller and disabled upon deactivation of the controller.

## Parameters

### Definition:
```{literalinclude} ../../dynaarm_controllers/src/freeze_controller_parameters.yaml
```

__arm_name__ | [Required]:\
Name of the arm as specified in the ros2control part of the urdf. Currently this is `${tf_prefix}DynaarmSystem`

__disable_at_deactivate__ | [Optional]:\
Allows to keep the arm in freeze mode after deactivation of the controller. __NOTE:__ this will lead into a system state where the arm won't accept any new commands!

### Example:

A full example can be found in the [dynaarm_demo](https://github.com/Duatic/dynaarm_demo/blob/main/dynaarm_examples/config/controllers.yaml) repository.

```yaml
freeze_controller:
    ros__parameters:
        arm_name: DynaarmSystem
```

## ROS Interfacing

None

## Additional Information

The freeze mode is implemented on hardware level by running the actuators in velocity control mode and commanding a 0 target velocity. This results in a "safe" freeze behaviour. It is possible to move the arm slightly by excerting sudden force impacts on the arm. As the arm is configured to run a 0 velocity the arm won't try to return into the initial freeze position.

## References

* [Source](https://github.com/Duatic/dynaarm_driver/blob/main/dynaarm_controllers/include/dynaarm_controllers/freeze_controller.hpp)

