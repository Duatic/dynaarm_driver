/*
 * CommandTranslator.hpp
 *
 *  Created on: Mar 16, 2018
 *      Author: Koen Krämer
 */

#pragma once

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

namespace dynaarm_driver
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
        static Eigen::Vector4d mapFromDynaarmToSerialCoordinates(const Eigen::Vector4d &input)
        {
            Eigen::Matrix4d mappingFromDynaarmToSerialAngles;
            mappingFromDynaarmToSerialAngles << 1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 1.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0;
            return mappingFromDynaarmToSerialAngles * input;
        }

        static Eigen::Vector4d mapFromDynaarmToSerialTorques(const Eigen::Vector4d &input)
        {
            Eigen::Matrix4d mappingFromSerialToDynaarmAngles;
            mappingFromSerialToDynaarmAngles << 1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, -1.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0;
            return mappingFromSerialToDynaarmAngles.transpose() * input;
        }

        /*!
         * Compute the mapping from the relative angles of the ocs2_convention to the absolute angles used for the dynaarm
         * theta = relative angles
         * q = absolute angles
         * q = mappingFromRelativeToAbsoluteAngles * theta
         */
        static Eigen::Vector4d mapFromSerialToDynaarmCoordinates(const Eigen::Vector4d &input)
        {
            Eigen::Matrix4d mappingFromSerialToDynaarmAngles;
            mappingFromSerialToDynaarmAngles << 1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, -1.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0;

            return mappingFromSerialToDynaarmAngles * input;
        }

        static Eigen::Vector4d mapFromSerialToDynaarmTorques(const Eigen::Vector4d &input)
        {
            Eigen::Matrix4d mappingFromDynaarmToSerialAngles;
            mappingFromDynaarmToSerialAngles << 1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 1.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0;
            return mappingFromDynaarmToSerialAngles.transpose() * input;
        }

        
    };

} // namespace
