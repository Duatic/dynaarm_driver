# Free Drive Controller

Controller that allows to freely move the arm around manually. \

```{note}
This controller has to be used in combination with the [Gravity Compensation Controller](./gravity_compensation_controller.md). Otherwise the arm will probably just fall down!
```

## Parameters

### Definition:
```{literalinclude} ../../dynaarm_controllers/src/freedrive_controller_parameters.yaml
```

__joints__ | [Required]:\
List of managed joints by the controller. Always needs to be a list of all joints of an arm in the order specified in the urdf

__d_gains__ | [Optional]:\
Allows to configure custom d-gains for the free drive mode. Higher gains mean more dampening.

### Example:

A full example can be found in the [dynaarm_demo](https://github.com/Duatic/dynaarm_demo/blob/main/dynaarm_examples/config/controllers.yaml) repository.

```yaml
freedrive_controller:
  ros__parameters:
    joints:
      - shoulder_rotation
      - shoulder_flexion
      - elbow_flexion
      - forearm_rotation
      - wrist_flexion
      - wrist_rotation
```

## ROS Interface

None

## Additional Information

The controller works by disableing the P and I gains of the motor controller running on each actuator. The D gain acts as a dampening against sudden motions. 
The [Gravity Compensation Controller](./gravity_compensation_controller.md) then simply provides a feed forward torque for each actuator that keeps the arm upright at its current position. By disturbing this stable state via a manual interaction it is possible to freely move the arm around.

## References

* [Source](https://github.com/Duatic/dynaarm_driver/blob/main/dynaarm_controllers/include/dynaarm_controllers/freedrive_controller.hpp)
* [Gravity Compensation Controller](./gravity_compensation_controller.md)