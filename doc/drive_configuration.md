# Actuator configuration

The DynaArm Driver provides a rather lowlevel access to the actuators of the arms.\
This allows the operator to fine tune the arm to the desired usecase.

The default configuration files for each actuator are located within the driver package:
[dynaarm_driver/dynaarm_driver/config](https://github.com/Duatic/dynaarm_driver/tree/main/dynaarm_driver/dynaarm_driver/config). They are separated for each version of the arm. 


## Providing custom configuration for an actuator

```{danger}
Using a different configuration that the one provided by duatic is done at your own risk.
The wrong parameterization might destroy an actuator or lead to uncontrolled behaviour.
```

In order to provide custom configuration files you can set the `drive_parameter_folder` in the instantiation of the `dynaarm` xacro macro in your description file.


### Example 

Adapted from the `dynaarm_standalone.urdf.xacro`:

```xml
  <xacro:dynaarm tf_prefix="$(arg tf_prefix)"
                parent_link="world"
                dof="$(arg dof)"
                mode="$(arg mode)"
                ethercat_bus="$(arg ethercat_bus)"
                covers="$(arg covers)"
                version="$(arg version)"
                drive_parameter_folder="my path to my own drive parameter files">
    <origin xyz="0 0 0.085" rpy="0.0 0.0 0.0" />
  </xacro:dynaarm>

```

```{tip}
You only need to provide configuration files for the actuators that you want to use a custom configuration for. 
For all other actuator the default configuration file is used!
```


```{toctree}
:hidden:
```
