#pragma once

#include <controller_interface/controller_interface.hpp>

/**
 * @brief By providing these additional types it is way easier to obtain in the on_activate method the right interfaces and manage them.
 */
namespace dynaarm_controllers {

/*Ordered interfaces*/
template<typename T>
using InterfaceReference = std::reference_wrapper<T>;
using CommandInterfaceReference = InterfaceReference<hardware_interface::LoanedCommandInterface>;
using StateInterfaceReference = InterfaceReference<hardware_interface::LoanedStateInterface>;

template<typename T>
using InterfaceReferences = std::vector<std::reference_wrapper<T>>;
using CommandInterfaceReferences = InterfaceReferences<hardware_interface::LoanedCommandInterface>;
using StateInterfaceReferences = InterfaceReferences<hardware_interface::LoanedStateInterface>;
}