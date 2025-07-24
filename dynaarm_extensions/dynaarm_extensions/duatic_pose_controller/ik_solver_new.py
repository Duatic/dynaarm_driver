import subprocess
import numpy as np
import pinocchio as pin
from numpy.linalg import norm, pinv


class IKSolver:
    def __init__(self, xacro_path, urdf_path, ee_frame_name="flange"):
        self.xacro_path = xacro_path
        self.urdf_path = urdf_path
        self.ee_frame_name = ee_frame_name

        # Convert xacro to urdf
        print("🔄 Converting xacro to URDF...")
        subprocess.run(["xacro", self.xacro_path, "-o", self.urdf_path], check=True)

        # Load model
        self.model, self.visual_model, self.collision_model = pin.buildModelsFromUrdf(self.urdf_path)
        self.data = self.model.createData()

        # Frame ID
        self.frame_id = self.model.getFrameId(self.ee_frame_name)
        if self.frame_id == 0:
            raise ValueError(f"Frame '{self.ee_frame_name}' not found in URDF.")

        # Joint limits
        margin = 0.01
        self.lower_bounds = self.model.lowerPositionLimit.copy() + margin
        self.upper_bounds = self.model.upperPositionLimit.copy() - margin

    def pose_from_list(self, pose_list):
        if len(pose_list) != 7:
            raise ValueError("Pose list must contain exactly 7 elements: [x, y, z, qx, qy, qz, qw]")
        translation = np.array(pose_list[:3])
        quat = pose_list[3:7]
        rotation = pin.Quaternion(quat[3], quat[0], quat[1], quat[2]).toRotationMatrix()
        return pin.SE3(rotation, translation)

    def damped_pinv(self, J, damping=0.05):
        JT = J.T
        return JT @ np.linalg.inv(J @ JT + damping**2 * np.eye(J.shape[0]))

    def solve_pose(self, pose_list, q_curr=None):
        # Desired pose
        oMdes = self.pose_from_list(pose_list)

        if q_curr is None:
            q_curr = pin.neutral(self.model)

        # FK
        pin.forwardKinematics(self.model, self.data, q_curr)
        pin.updateFramePlacements(self.model, self.data)
        current_pose = self.data.oMf[self.frame_id]

        # Error in Cartesian space
        pose_error_se3 = pin.log(oMdes.actInv(current_pose)).vector  # 6D error (rot + trans)
        
        # Impedance control in Cartesian space
        kp = 40.0
        kd = 2.0
        ee_vel = np.zeros(6)  # for now

        f_impedance_cmd = kp * pose_error_se3 - kd * ee_vel

        # Reorder (flip torque/force)
        f_flipped = np.zeros(6)
        f_flipped[:3] = f_impedance_cmd[3:]  # angular
        f_flipped[3:] = f_impedance_cmd[:3]  # linear

        # Jacobian at current q
        #pin.computeJointPlacements(self.model, self.data)
        pin.computeFrameJacobian(self.model, self.data, q_curr, self.frame_id, pin.ReferenceFrame.LOCAL)
        J = pin.getFrameJacobian(self.model, self.data, self.frame_id, pin.ReferenceFrame.LOCAL)

        # Jacobian pseudo-inverse
        J_pinv = self.damped_pinv(J)

        # Joint delta from impedance force
        q_delta_task = J_pinv @ f_flipped

        # Null-space correction to only move joint 6 (if needed)
        task_matrix = np.zeros((1, self.model.nv))
        task_matrix[0, min(5, self.model.nv - 1)] = 1.0  # just ensure safe indexing

        N1 = np.eye(self.model.nv) - J_pinv @ J
        q_null = self.damped_pinv(task_matrix @ N1) @ (-task_matrix @ q_delta_task)
        q_delta = q_delta_task + N1 @ q_null

        # Scaling and limit
        q_delta *= 1.0 / 70.0
        max_step = 0.1
        if np.max(np.abs(q_delta)) > max_step:
            q_delta = max_step * q_delta / np.max(np.abs(q_delta))

        # Apply delta
        q_next = q_curr + q_delta
        q_next = np.clip(q_next, self.lower_bounds, self.upper_bounds)

        # Compute new pose
        pin.forwardKinematics(self.model, self.data, q_next)
        pin.updateFramePlacements(self.model, self.data)
        new_pose = self.data.oMf[self.frame_id]

        position_error = np.linalg.norm(oMdes.translation - new_pose.translation)

        return q_next, {
            'position_error': position_error,
            'desired_pose': oMdes,
            'achieved_pose': new_pose
        }

    def forward_kinematics(self, joint_angles):
        q = np.asarray(joint_angles)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        ee_pose = self.data.oMf[self.frame_id]

        position = ee_pose.translation.tolist()
        quat = pin.Quaternion(ee_pose.rotation)
        return position + [quat.x, quat.y, quat.z, quat.w]
