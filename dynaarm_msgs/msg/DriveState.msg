# Timestamp
builtin_interfaces/Time timestamp

# The configured drive name
string name
# The id of the drive on the bus
int32 id
# Current joint position [rad]
float64 joint_position
# Current joint velocity [rad/s]
float64 joint_velocity
# Current joint torque [Nm]
float64 joint_effort

# Currently commanded joint position [rad]
float64 joint_position_commanded
# Currently commanded joint velocity [rad/s]
float64 joint_velocity_commanded
# Currently commanded joint torque [Nm]
float64 joint_effort_commanded


#TODO add commanded values?
#TODO refactor into using JointState message?

# Current system temperature [°C]
float32 temperature_system
# Current phase A temperature [°C]
float32 temperature_phase_a
# Current phase B temperature [°C]
float32 temperature_phase_b
# Current phase C temperature [°C]
float32 temperature_phase_c

# Current bus voltage [V]
float32 bus_voltage

#TODO add drive state, warnings
