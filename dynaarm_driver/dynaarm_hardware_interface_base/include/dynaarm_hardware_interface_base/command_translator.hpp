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
    if (size < 4) {
      throw std::invalid_argument("Input vector must have at least 4 elements");
    }

    // Create a dynamic matrix with size 'size'
    Eigen::MatrixXd mapping = Eigen::MatrixXd::Identity(size, size);

    // Modify the first 4x4 submatrix
    mapping.block<4, 4>(0, 0) << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    // Return the transformed vector
    return mapping * input;
  }

  static Eigen::VectorXd mapFromDynaarmToSerialTorques(const Eigen::VectorXd& input)
  {
    int size = input.size();
    if (size < 4) {
      throw std::invalid_argument("Input vector must have at least 4 elements");
    }

    // Create a dynamic matrix with size 'size'
    Eigen::MatrixXd mapping = Eigen::MatrixXd::Identity(size, size);

    // Modify the first 4x4 submatrix
    mapping.block<4, 4>(0, 0) << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;

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
    if (size < 4) {
      throw std::invalid_argument("Input vector must have at least 4 elements");
    }

    // Create a dynamic matrix with size 'size'
    Eigen::MatrixXd mapping = Eigen::MatrixXd::Identity(size, size);

    // Modify the first 4x4 submatrix
    mapping.block<4, 4>(0, 0) << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    // Return the transformed vector
    return mapping * input;
  }

  static Eigen::VectorXd mapFromSerialToDynaarmTorques(const Eigen::VectorXd& input)
  {
    int size = input.size();
    if (size < 4) {
      throw std::invalid_argument("Input vector must have at least 4 elements");
    }

    // Create a dynamic matrix with size 'size'
    Eigen::MatrixXd mapping = Eigen::MatrixXd::Identity(size, size);

    // Modify the first 4x4 submatrix
    mapping.block<4, 4>(0, 0) << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    // Return the transformed vector
    return mapping * input;
  }
};

}  // namespace dynaarm_hardware_interface_common
