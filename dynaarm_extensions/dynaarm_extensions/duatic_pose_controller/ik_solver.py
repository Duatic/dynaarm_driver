import subprocess
import numpy as np
import pinocchio as pin
from scipy.optimize import minimize
from numpy.linalg import norm


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


    @staticmethod
    def pose_from_list(pose_list):
        """
        Convert a list of pose values to a Pinocchio SE3 object.
        """
        if len(pose_list) != 7:
            raise ValueError("Pose list must contain exactly 7 elements: [x, y, z, roll, pitch, yaw, w]")
        
        translation = np.array(pose_list[:3])

        # The rotation is given as a quaternion (x, y, z, w)
        quat = pose_list[3:7]
        rotation = pin.Quaternion(quat[3], quat[0], quat[1], quat[2]).toRotationMatrix()

        return pin.SE3(rotation, translation)

    def solve_pose(self, pose_list, q_init=None):
        oMdes = self.pose_from_list(pose_list)

        def pose_error(q_vec):
            q = np.asarray(q_vec)
            if np.any(q < self.lower_bounds) or np.any(q > self.upper_bounds):
                return 1e3
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            current_pose = self.data.oMf[self.frame_id]
            err_vec = pin.log(oMdes.actInv(current_pose)).vector
            return norm(err_vec)

        if q_init is None:
            q_init = pin.neutral(self.model)

        bounds = list(zip(self.lower_bounds, self.upper_bounds))
        res = minimize(
            pose_error,
            q_init,
            method='SLSQP',
            bounds=bounds,
            tol=1e-10,
            options={'disp': False, 'maxiter': 1000}
        )

        if res.success:
            q_sol = res.x
            pin.forwardKinematics(self.model, self.data, q_sol)
            pin.updateFramePlacements(self.model, self.data)
            achieved_pose = self.data.oMf[self.frame_id]
            position_error = np.linalg.norm(oMdes.translation - achieved_pose.translation)
            return q_sol, {
                'position_error': position_error,
                'desired_pose': oMdes,
                'achieved_pose': achieved_pose
            }
        else:
            return None, None
        
    def forward_kinematics(self, joint_angles):
        """
        Compute the forward kinematics for the given joint angles.

        Args:
            joint_angles: Array-like of joint angles.

        Returns:
            List of 7 elements: [x, y, z, qx, qy, qz, qw]
        """
        q = np.asarray(joint_angles)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        ee_pose = self.data.oMf[self.frame_id]

        position = ee_pose.translation.tolist()
        rotation_matrix = ee_pose.rotation
        quat = pin.Quaternion(rotation_matrix)
        quaternion = [quat.x, quat.y, quat.z, quat.w]

        return position + quaternion


if __name__ == "__main__":

    ik = IKSolver(
            xacro_path="/ros2_ws/src/dynaarm_demo/dynaarm_single_example/dynaarm_single_example_description/urdf/dynaarm_single_example.urdf.xacro",
            urdf_path="config/robot.urdf",
            ee_frame_name="flange"
    )

    pose = [0.2, 0.1, 0.7, 0.0, 0.2, 1.0, 0.0]

    solutions = ik.solve_pose(pose)

    print(solutions)