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

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np
from tf_transformations import quaternion_matrix


class DuaticMarkerHelper:

    def __init__(self, node, namespace_prefix: str = ""):
        self.node = node
        self.marker_publisher = self.node.create_publisher(MarkerArray, "/marker_visu", 100)
        self.namespace_prefix = namespace_prefix

    def create_pose_markers(self, pose_stamped, frame, ns_prefix=""):
        """Create a sphere marker at the pose and three arrow markers for the axes."""
        marker_array = MarkerArray()

        # Sphere marker
        sphere_marker = Marker()
        sphere_marker.header.frame_id = frame
        sphere_marker.header.stamp = pose_stamped.header.stamp
        sphere_marker.ns = f"{ns_prefix}sphere"
        sphere_marker.id = 1001
        sphere_marker.type = Marker.SPHERE
        sphere_marker.action = Marker.ADD
        sphere_marker.pose = pose_stamped.pose
        sphere_marker.scale.x = 0.025
        sphere_marker.scale.y = 0.025
        sphere_marker.scale.z = 0.025
        sphere_marker.color.r = 1.0
        sphere_marker.color.g = 0.0
        sphere_marker.color.b = 0.0
        sphere_marker.color.a = 1.0
        marker_array.markers.append(sphere_marker)

        # Axes as arrows
        q = pose_stamped.pose.orientation
        T = quaternion_matrix([q.x, q.y, q.z, q.w])
        origin = np.array(
            [
                pose_stamped.pose.position.x,
                pose_stamped.pose.position.y,
                pose_stamped.pose.position.z,
            ]
        )

        axes = [
            (np.array([1, 0, 0]), [1.0, 0.0, 0.0, 1.0], 1002),  # X axis, red
            (np.array([0, 1, 0]), [0.0, 1.0, 0.0, 1.0], 1003),  # Y axis, green
            (np.array([0, 0, 1]), [0.0, 0.0, 1.0, 1.0], 1004),  # Z axis, blue
        ]

        for axis_vec, color, marker_id in axes:
            direction = T[:3, :3] @ axis_vec
            end = origin + direction * 0.1

            arrow_marker = Marker()
            arrow_marker.header.frame_id = frame
            arrow_marker.header.stamp = pose_stamped.header.stamp
            arrow_marker.ns = f"{ns_prefix}axis_{marker_id}"
            arrow_marker.id = marker_id
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            arrow_marker.scale.x = 0.01  # shaft length
            arrow_marker.scale.y = 0.02  # shaft diameter
            arrow_marker.scale.z = 0.025  # head diameter
            arrow_marker.color.r = color[0]
            arrow_marker.color.g = color[1]
            arrow_marker.color.b = color[2]
            arrow_marker.color.a = color[3]
            arrow_marker.points = [
                Point(x=origin[0], y=origin[1], z=origin[2]),
                Point(x=end[0], y=end[1], z=end[2]),
            ]
            marker_array.markers.append(arrow_marker)

        self.marker_publisher.publish(marker_array)

    def clear_markers(self):
        """Clears all or specific markers."""
        marker_array = MarkerArray()
        marker = Marker()
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)
        self.marker_publisher.publish(marker_array)
