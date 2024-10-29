#pragma once

#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace dynaarm_hardware_interface_common
{
    struct JointState
    {
        std::string name;
        double position = 0.0;
        double velocity = 0.0;
        double effort = 0.0;
    };

    struct JointCommand
    {
        std::string name;
        double position = 0.0;
        double velocity = 0.0;
        double effort = 0.0;
        double p_gain = 0.0;
        double i_gain = 0.0;
        double d_gain = 0.0;
        double command_freeze_mode = 1.0; // start in freeze mode
    };

    struct MotorState
    {
        std::string name;
        double position = 0.0;
        double velocity = 0.0;
        double effort = 0.0;

        double bus_voltage = 0.0;

        double temperature = 0.0;
        double temperature_coil_A = 0.0;
        double temperature_coil_B = 0.0;
        double temperature_coil_C = 0.0;
    };

    struct MotorCommand
    {
        std::string name;
        double position = 0.0;
        double velocity = 0.0;
        double effort = 0.0;
        double p_gain = 0.0;
        double i_gain = 0.0;
        double d_gain = 0.0;
        double command_freeze_mode = 1.0; // start in freeze mode
    };

} // namespace dynaarm_hardware_interface_common