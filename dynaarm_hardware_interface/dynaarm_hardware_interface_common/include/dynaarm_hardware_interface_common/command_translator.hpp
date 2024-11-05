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

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

namespace dynaarm_hardware_interface_common
{

class CommandTranslator
{
public:
  CommandTranslator() = default;

  virtual ~CommandTranslator() = default;

  /*!
   * Compute the mapping from the absolute angles of the dynaarm to the relative angles used in the Ocs2 convention.
   * theta = relative angles
   * q = absolute angles
   * theta = mappingFromAbsoluteToRelativeAngles * q
   */
  static Eigen::VectorXd mapFromDynaarmToSerialCoordinates(const Eigen::VectorXd& input)
  {
    int size = input.size();
    if (size < 4)
    {
      throw std::invalid_argument("Input vector must have at least 4 elements");
    }

    // Create a dynamic matrix with size 'size'
    Eigen::MatrixXd mapping = Eigen::MatrixXd::Identity(size, size);

    // Modify the first 4x4 submatrix
    mapping.block<4, 4>(0, 0) << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    // Return the transformed vector
    return mapping * input;
  }

  static Eigen::VectorXd mapFromDynaarmToSerialTorques(const Eigen::VectorXd& input)
  {
    int size = input.size();
    if (size < 4)
    {
      throw std::invalid_argument("Input vector must have at least 4 elements");
    }

    // Create a dynamic matrix with size 'size'
    Eigen::MatrixXd mapping = Eigen::MatrixXd::Identity(size, size);

    // Modify the first 4x4 submatrix
    mapping.block<4, 4>(0, 0) << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    // Return the transformed vector
    return mapping * input;
  }

  /*!
   * Compute the mapping from the relative angles of the ocs2_convention to the absolute angles used for the dynaarm
   * theta = relative angles
   * q = absolute angles
   * q = mappingFromRelativeToAbsoluteAngles * theta
   */
  static Eigen::VectorXd mapFromSerialToDynaarmCoordinates(const Eigen::VectorXd& input)
  {
    int size = input.size();
    if (size < 4)
    {
      throw std::invalid_argument("Input vector must have at least 4 elements");
    }

    // Create a dynamic matrix with size 'size'
    Eigen::MatrixXd mapping = Eigen::MatrixXd::Identity(size, size);

    // Modify the first 4x4 submatrix
    mapping.block<4, 4>(0, 0) << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    // Return the transformed vector
    return mapping * input;
  }

  static Eigen::VectorXd mapFromSerialToDynaarmTorques(const Eigen::VectorXd& input)
  {
    int size = input.size();
    if (size < 4)
    {
      throw std::invalid_argument("Input vector must have at least 4 elements");
    }

    // Create a dynamic matrix with size 'size'
    Eigen::MatrixXd mapping = Eigen::MatrixXd::Identity(size, size);

    // Modify the first 4x4 submatrix
    mapping.block<4, 4>(0, 0) << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    // Return the transformed vector
    return mapping * input;
  }
};

}  // namespace dynaarm_hardware_interface_common
