from .Rotation import Rotation as R
import numpy as np

# Intrinsic ZYX euler angles are yaw pitch roll
# https://en.wikipedia.org/wiki/Euler_angles#Conventions
# Intrinsic euler angles are defined using capital letters in scipy
# https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.as_euler.html#

# Segment frame convention: x+ right, y+ up, z+ posterior
FLEXION_AXIS_PARENT = np.array([1.0, 0.0, 0.0], dtype=float)  # +X

class IntrinsicZYXEuler:
    def __init__(self, rot):
        euler = rot.as_euler("ZYX", degrees=True)
        self.yaw, self.pitch, self.roll = euler[0] if not rot.single else euler

    def __str__(self):
        return f"Yaw = {self.yaw}, Pitch = {self.pitch}, Roll = {self.roll}"

    def __repr__(self):
        return f"IntrinsicZYXEuler(Yaw = {self.yaw}, Pitch = {self.pitch}, Roll = {self.roll})"


class JointAngles:
    def __init__(self, isRightLeg=True):
        """
        Initializes the JointAngles class.

        Args:
            isRightLeg (bool): Indicates whether the leg is the right leg or not.
        """

        # sensor to segment alignment quaternion, inv denotes conjugate.
        self.BS_q_pelvis_inv = None
        self.BS_q_thigh_inv = None
        self.BS_q_shank_inv = None
        self.BS_q_foot_inv = None


    def calibrate(self, foot_quat, pelvis_quat, thigh_quat, shank_quat):
        """
        Performs sensor to segment calibration.

        Args:
            foot_quat (Rotation): Rotation representing the foot orientation.
            pelvis_quat (Rotation): Rotation representing the pelvis orientation.
            thigh_quat (Rotation): Rotation representing the thigh orientation.
            shank_quat (Rotation): Rotation representing the shank orientation.
        """

        # Define the calibration reference by the pelvis facing direction.
        # Pelvis IMU axes: y up, z posterior (matches segment axes), so we use pelvis_quat as target.
        GB_q0_target = pelvis_quat

        def initialize_quat_inv(this_quat):
            # Full 3D sensor->segment mapping at the calibration pose:
            # R_S->B = (R_G->S)^-1 * (R_G->B)
            # Here, R_G->B is defined by the pelvis orientation in the calibration pose.
            return this_quat.inv() * GB_q0_target

        self.BS_q_pelvis_inv = initialize_quat_inv(pelvis_quat)
        self.BS_q_thigh_inv = initialize_quat_inv(thigh_quat)
        self.BS_q_shank_inv = initialize_quat_inv(shank_quat)
        self.BS_q_foot_inv = initialize_quat_inv(foot_quat)

        print("Hip, knee and ankle all angles Calibrate finished")

    @staticmethod
    def calculate_GB_quat(GS_quat, bs_inv_quat):
        # This method calculates the quaternion relative to the body segment.
        GB_quat = GS_quat * bs_inv_quat
        return GB_quat

    @staticmethod
    def _wrap_deg(angle_deg: float) -> float:
        return (angle_deg + 180) % 360 - 180

    @staticmethod
    def _twist_angle_about_axis(q_rel, axis_parent_xyz: np.ndarray) -> float:
        """
        Return signed twist angle (degrees) of q_rel about axis expressed in the PARENT frame.

        q_rel should be: q_parent.inv() * q_child  (child w.r.t parent, expressed in parent frame)
        axis_parent_xyz is the twist axis in parent coordinates (unit vector).
        """
        a = np.asarray(axis_parent_xyz, dtype=float)
        n = np.linalg.norm(a)
        if n == 0:
            raise ValueError("axis_parent_xyz must be non-zero")
        a = a / n

        # SciPy Rotation.as_quat() convention: [x, y, z, w]
        q = q_rel.as_quat()
        v = q[:3]
        w = q[3]

        # Project vector part onto axis to isolate twist
        v_par = a * float(np.dot(v, a))

        twist = np.array([v_par[0], v_par[1], v_par[2], w], dtype=float)
        twist_norm = np.linalg.norm(twist)
        if twist_norm < 1e-12:
            return 0.0
        twist /= twist_norm

        v_t = twist[:3]
        w_t = twist[3]

        # Signed angle from signed sin(half-angle) along axis
        s = float(np.dot(v_t, a))
        angle_rad = 2.0 * np.arctan2(abs(s), w_t)
        angle_deg = np.degrees(angle_rad)
        if s < 0:
            angle_deg = -angle_deg

        return (angle_deg + 180.0) % 360.0 - 180.0

    def calculate_Hip_Flex(self, pelvis_quat, thigh_quat):
        GB_pelvis_q = self.calculate_GB_quat(pelvis_quat, self.BS_q_pelvis_inv)
        GB_thigh_q = self.calculate_GB_quat(thigh_quat, self.BS_q_thigh_inv)
        q_rel = GB_pelvis_q.inv() * GB_thigh_q
        return self._twist_angle_about_axis(q_rel, self.FLEXION_AXIS_PARENT)


    def calculate_Knee_Flex(self, thigh_quat, shank_quat):
        GB_thigh_q = self.calculate_GB_quat(thigh_quat, self.BS_q_thigh_inv)
        GB_shank_q = self.calculate_GB_quat(shank_quat, self.BS_q_shank_inv)
        q_rel = GB_thigh_q.inv() * GB_shank_q
        return -self._twist_angle_about_axis(q_rel, self.FLEXION_AXIS_PARENT)

    def calculate_Ankle_Flex(self, shank_quat, foot_quat):
        GB_shank_q = self.calculate_GB_quat(shank_quat, self.BS_q_shank_inv)
        GB_foot_q = self.calculate_GB_quat(foot_quat, self.BS_q_foot_inv)
        q_rel = GB_shank_q.inv() * GB_foot_q
        return self._twist_angle_about_axis(q_rel, self.FLEXION_AXIS_PARENT)
