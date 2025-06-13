/*
 * Copyright 2024 Duatic AG
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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
  double acceleration = 0.0;
  double effort = 0.0;

  double position_last = NAN;

  double position_commanded = 0.0;
  double velocity_commanded = 0.0;
  double acceleration_commanded = 0.0;
  double effort_commanded = 0.0;
};

struct JointCommand
{
  std::string name;
  double position = 0.0;
  double velocity = 0.0;
  double acceleration = 0.0;
  double effort = 0.0;
  double p_gain = 0.0;
  double i_gain = 0.0;
  double d_gain = 0.0;
};

struct MotorState
{
  std::string name;
  double position = 0.0;
  double velocity = 0.0;
  double acceleration = 0.0;
  double effort = 0.0;

  double bus_voltage = 0.0;

  double temperature = 0.0;
  double temperature_coil_A = 0.0;
  double temperature_coil_B = 0.0;
  double temperature_coil_C = 0.0;

  double position_commanded = 0.0;
  double velocity_commanded = 0.0;
  double effort_commanded = 0.0;
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
};

}  // namespace dynaarm_hardware_interface_common
