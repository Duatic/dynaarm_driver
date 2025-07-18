# Copyright 2025 Duatic AG
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions, and
#    the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions, and
#    the following disclaimer in the documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or
#    promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rclpy
from rclpy.parameter_client import AsyncParameterClient


class DuaticParamHelper:
    """Helper class for Joint Trajectory Controller topic discovery and management."""

    def __init__(self, node):
        self.node = node

    def get_param_values(self, controller_ns, param_name):
        """Retrieve parameter values from the node."""
        param_client = AsyncParameterClient(self.node, controller_ns)
        param_client.wait_for_services(timeout_sec=5.0)

        future = param_client.get_parameters([param_name])

        self.node.get_logger().debug(f"Requesting parameter {param_name} from {controller_ns}")
        while not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.05)

        self.node.get_logger().debug(
            f"Parameter {param_name} retrieval completed for {controller_ns}"
        )

        if future.result() is not None and future.result().values:  # type: ignore
            return future.result().values  # type: ignore

        return None
