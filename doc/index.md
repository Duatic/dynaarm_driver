# DynaArm Driver

This package contains the actual ros2control driver for the DynaArm as well as [additional controllers](./controllers.md) needed touse the full potential of your DynaArm.


## Providing custom configuration for an actuator

The DynaArm Driver provides a rather lowlevel access to the actuators of the arms.\
This allows the operator to fine tune the arm to the desired usecase.

:::{danger}
Using a different configuration that the one provided by duatic is done at your own risk.
The wrong parameterization might destroy an actuator or lead to uncontrolled behaviour.
:::



```{toctree}
:hidden:
./controllers.md
```
