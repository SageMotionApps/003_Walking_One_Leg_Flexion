from .Rotation import Rotation as R

# Intrinsic ZYX euler angles are yaw pitch roll
# https://en.wikipedia.org/wiki/Euler_angles#Conventions
# Intrinsic euler angles are defined using capital letters in scipy
# https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.as_euler.html#


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

        # The yaw offset depends on the leg. The Right leg's yaw is rotated the other way.
        self.yaw_offset = -90 if isRightLeg else 90

    def calibrate(self, foot_quat, pelvis_quat, thigh_quat, shank_quat):
        """
        Performs sensor to segment calibration.

        Args:
            foot_quat (Rotation): Rotation representing the foot orientation.
            pelvis_quat (Rotation): Rotation representing the pelvis orientation.
            thigh_quat (Rotation): Rotation representing the thigh orientation.
            shank_quat (Rotation): Rotation representing the shank orientation.
        """

        # Get body segment quaternions relative to the target.
        # Pelvis does not include the offset since it is the base.
        # We dont include the offset for the foot since it does not rotate.
        self.BS_q_pelvis_inv = self.initialize_quat_inv(pelvis_quat, False)
        self.BS_q_thigh_inv = self.initialize_quat_inv(thigh_quat, True)
        self.BS_q_shank_inv = self.initialize_quat_inv(shank_quat, True)
        self.BS_q_foot_inv = self.initialize_quat_inv(foot_quat, False)

        print("Hip, knee and ankle all angles Calibrate finished")

    def initialize_quat_inv(self, this_quat, include_offset):
        """
        Initializes the conjugate quaternion for sensor to segment alignment.

        Args:
            this_quat (Rotation): Rotation representing the sensor orientation.

        Returns:
            Rotation: Conjugate Rotation representing the alignment target.
        """
        R_this_init_Yaw = IntrinsicZYXEuler(this_quat).yaw
        if include_offset:
            R_this_init_Yaw = R_this_init_Yaw + self.yaw_offset

        R_this_init_Yaw = (R_this_init_Yaw + 180) % 360 - 180

        GB_q0_target = R.from_euler(
            seq="ZYX", angles=[R_this_init_Yaw, 0, 0], degrees=True
        )

        this_quat = this_quat.inv()
        return this_quat * GB_q0_target

    def calculate_GB_quat(self, GS_quat, bs_inv_quat):
        # This method calculates the quaternion relative to the body segment.
        GB_quat = GS_quat * bs_inv_quat
        return GB_quat

    def calculate_Hip_Flex(self, pelvis_quat, thigh_quat):
        GB_pelvis_q = self.calculate_GB_quat(pelvis_quat, self.BS_q_pelvis_inv)
        GB_thigh_q = self.calculate_GB_quat(thigh_quat, self.BS_q_thigh_inv)

        pelvis_angles = IntrinsicZYXEuler(GB_pelvis_q)
        thigh_angles = IntrinsicZYXEuler(GB_thigh_q)
        Hip_flex = pelvis_angles.roll - thigh_angles.roll
        Hip_flex = (Hip_flex + 180) % 360 - 180
        return Hip_flex

    def calculate_Knee_Flex(self, thigh_quat, shank_quat):
        GB_thigh_q = self.calculate_GB_quat(thigh_quat, self.BS_q_thigh_inv)
        GB_shank_q = self.calculate_GB_quat(shank_quat, self.BS_q_shank_inv)

        thigh_angles = IntrinsicZYXEuler(GB_thigh_q)
        shank_angles = IntrinsicZYXEuler(GB_shank_q)
        Knee_flex = -(thigh_angles.roll - shank_angles.roll)
        Knee_flex = (Knee_flex + 180) % 360 - 180
        return Knee_flex

    def calculate_Ankle_Flex(self, shank_quat, foot_quat):
        GB_shank_q = self.calculate_GB_quat(shank_quat, self.BS_q_shank_inv)
        GB_foot_q = self.calculate_GB_quat(foot_quat, self.BS_q_foot_inv)

        shank_angles = IntrinsicZYXEuler(GB_shank_q)
        foot_angles = IntrinsicZYXEuler(GB_foot_q)
        Ankle_flex = -(shank_angles.roll - foot_angles.roll)
        Ankle_flex = (Ankle_flex + 180) % 360 - 180
        return Ankle_flex
