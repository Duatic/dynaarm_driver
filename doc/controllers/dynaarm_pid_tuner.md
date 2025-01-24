# DynaArm PID Tuner

Controller that allows to tune the PID parameters of the controllers running on the actuators on the live system.\

```{important}
Please use with care. By selecting bad parameters it is possible to damage the hardware.
```


## Parameters

### Definition:
```{literalinclude} ../../dynaarm_controllers/src/dynaarm_pid_tuner_parameters.yaml
```

__joints__ | [Required]:\
List of managed joints by the controller. Always needs to be a list of all joints of an arm in the order specified in the urdf


### Example:

A full example can be found in the [dynaarm_demo](https://github.com/Duatic/dynaarm_demo/blob/main/dynaarm_examples/config/controllers.yaml) repository.

```yaml
pid_tuner:
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

__~/<joint name>/pid_gains/set__ | [Subscription]:\
type: [<dynaarm_msgs/msg/PIDGains](https://github.com/Duatic/dynaarm_driver/blob/main/dynaarm_msgs/msg/PIDGains.msg).\
For each configured joint a new topic is created that awaits new PID-gains. \
The received gains are then configured as a parameters of the controller node and set to the drive during the next update cycle.


## Additional Information

During controller activation the current gains are set as parameter to the controller ROS node.
They can be access via `ros2 param get <controller node name> <joint name>/<gain type>` with gain type being one of `[p_gain, i_gain,d_gain]`.

## References

* [Source](https://github.com/Duatic/dynaarm_driver/blob/main/dynaarm_controllers/include/dynaarm_controllers/dynaarm_pid_tuner.hpp)
