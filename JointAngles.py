from .Rotation import Rotation as R
import numpy as np


# Segment frame convention: x- right, y+ up, z- posterior
FLEXION_AXIS_PARENT = np.array([-1.0, 0.0, 0.0], dtype=float)  # -X
VERTICAL_AXIS = np.array([0.0, 0.0, 1.0], dtype=float)  # Z

def wrap_deg(angle_deg: float) -> float:
    return (angle_deg + 180) % 360 - 180

def get_swing_twist_decomposition(q: R, v: np.ndarray) -> (R, R):
    """
    Decompose rotation q into swing and twist, e.g. elbow flexion and pronation, about the reference axis v (in the same frame).

    Returns (qs, qt) such that:
        q = qs * qt
    where:
        - qs (swing) rotates v to w = q.apply(v) without twisting about v
        - qt (twist) is a pure rotation about axis v

    Notes:
        - v must be non-zero; it will be normalized internally.
        - Axis v, the twist axis, is orthogonal to the swing axis.
        - Source: https://arxiv.org/pdf/1506.05481
    """
    v = np.asarray(v, dtype=float)
    v_norm = np.linalg.norm(v)
    if v_norm == 0.0:
        raise ValueError("v must be non-zero")
    v = v / v_norm

    # Rotate reference axis by q (active rotation)
    w = q.apply(v)

    # Compute swing axis n (perpendicular to both v and w)
    n = np.cross(v, w)
    n_norm = np.linalg.norm(n)

    # Robust angle between v and w:
    # sin(alpha) = ||v×w|| / ||v|| ||w||
    # cos(alpha) = v·w / ||v|| ||w||
    # tan(alpha) = sin(alpha) / cos(alpha)
    # tan(alpha) = ||v×w|| / v·w
    # alpha = atan2(||v×w||, v·w)
    d = float(np.dot(v, w))
    d = max(-1.0, min(1.0, d))
    alpha = float(np.arctan2(n_norm, d))

    if n_norm < 1e-12:
        # v and w are parallel or anti-parallel (sign flip parallel): swing is either identity (alpha~0)
        # or a 180° rotation about any axis orthogonal to v (alpha~pi).
        if d > 0.0:
            qs = R.identity()
        else:
            # Choose a stable axis orthogonal to v
            # ortho · v = 0 => ortho is orthogonal to v

            # pick the smallest component to avoid near-parallel
            if abs(v[0]) < abs(v[1]) and abs(v[0]) < abs(v[2]):
                ortho = np.array([0.0, -v[2], v[1]])
                # [0, -v2, v1] · [v0, v1, v2] = 0 => ortho is orthogonal to v
            elif abs(v[1]) < abs(v[2]):
                ortho = np.array([-v[2], 0.0, v[0]])
                # [-v2, 0, v0] · [v0, v1, v2] = 0 => ortho is orthogonal to v
            else:
                ortho = np.array([-v[1], v[0], 0.0])
                # [-v1, v0, 0] · [v0, v1, v2] = 0 => ortho is orthogonal to v
            ortho /= np.linalg.norm(ortho)
            qs = R.from_rotvec(ortho * np.pi)
    else:
        n = n / n_norm
        qs = R.from_rotvec(n * alpha)

    # Twist-after-swing: q = qs * qt  =>  qt = qs^{-1} * q
    qt = qs.inv() * q
    return qs, qt

def signed_twist_angle_deg(qt: R, v: np.ndarray) -> float:
    """
    Signed twist angle (degrees) about axis v.
    qt must be a pure twist about v.
    """
    v = np.asarray(v, dtype=float)
    v /= np.linalg.norm(v)

    rotvec = qt.as_rotvec()  
    angle_rad = np.linalg.norm(rotvec)
    if angle_rad < 1e-12:
        return 0.0

    axis = rotvec / angle_rad
    sign = np.sign(np.dot(axis, v))
    return np.degrees(angle_rad) * sign

def _unit(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if n == 0.0:
        raise ValueError("zero-length vector")
    return v / n

def _project_onto_plane(v: np.ndarray, normal: np.ndarray) -> np.ndarray:
    """
    Remove the component of v along normal. Returns a vector in the plane orthogonal to normal.
    """
    normal = _unit(normal)
    return v - normal * np.dot(normal, v)

def _normalize_or_none(v: np.ndarray) -> np.ndarray|None:
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if n < 1e-12:
        return None
    return v / n

def signed_angle_about_axis(v_from: np.ndarray, v_to: np.ndarray, axis: np.ndarray) -> float:
    """
    Signed angle (degrees) from v_from to v_to about axis.
    Vectors are projected onto the plane orthogonal to axis.
    """
    axis = _unit(axis)
    v1 = _project_onto_plane(v_from, axis)
    v2 = _project_onto_plane(v_to, axis)
    n1 = _normalize_or_none(v1)
    n2 = _normalize_or_none(v2)
    if n1 is None or n2 is None:
        return 0.0
    angle = np.arctan2(np.dot(axis, np.cross(v1, v2)), np.dot(v1, v2))
    return np.degrees(angle)

class JointAngles:
    def __init__(self):
        """
        Initializes the JointAngles class.

        """
        # sensor to segment alignment quaternion, inv denotes conjugate.
        self.BS_q_pelvis_inv = None
        self.BS_q_thigh_inv = None
        self.BS_q_shank_inv = None
        self.BS_q_foot_inv = None
        self.hip_flex_offset_deg = 0.0
        self.hip_side_sign = 1.0

    def set_leg_is_right(self, is_right: bool) -> None:
        self.hip_side_sign = 1.0 if is_right else -1.0


    def calibrate(self, foot_quat, pelvis_quat, thigh_quat, shank_quat):
        """
        Performs sensor to segment calibration.

        Args:
            foot_quat (Rotation): Rotation representing the foot orientation.
            pelvis_quat (Rotation): Rotation representing the pelvis orientation.
            thigh_quat (Rotation): Rotation representing the thigh orientation.
            shank_quat (Rotation): Rotation representing the shank orientation.
        """

        # Pelvis keeps its natural tilt; other segments are gravity-up with pelvis yaw.
        pelvis_yaw_deg, _, _ = pelvis_quat.as_euler("ZYX", degrees=True)
        GB_q0_target_pelvis = pelvis_quat
        GB_q0_target_segments = R.from_euler("Z", pelvis_yaw_deg, degrees=True)

        self.BS_q_pelvis_inv = pelvis_quat.inv() * GB_q0_target_pelvis
        self.BS_q_thigh_inv = thigh_quat.inv() * GB_q0_target_segments
        self.BS_q_shank_inv = shank_quat.inv() * GB_q0_target_segments
        self.BS_q_foot_inv = foot_quat.inv() * GB_q0_target_segments

        GB_pelvis_q = pelvis_quat * self.BS_q_pelvis_inv
        GB_thigh_q = thigh_quat * self.BS_q_thigh_inv
        q_rel = GB_pelvis_q.inv() * GB_thigh_q
        _, twist = get_swing_twist_decomposition(q_rel, FLEXION_AXIS_PARENT)
        self.hip_flex_offset_deg = signed_twist_angle_deg(
            twist, FLEXION_AXIS_PARENT
        )

        print("Hip, knee and ankle all angles Calibrate finished")


    def calculate_Hip_Flex(self, pelvis_quat, thigh_quat):
        GB_pelvis_q = pelvis_quat * self.BS_q_pelvis_inv
        GB_thigh_q = thigh_quat * self.BS_q_thigh_inv
        q_rel = GB_pelvis_q.inv() * GB_thigh_q
        _, twist = get_swing_twist_decomposition(q_rel, FLEXION_AXIS_PARENT)
        angle = signed_twist_angle_deg(twist, FLEXION_AXIS_PARENT)
        return wrap_deg(angle - self.hip_flex_offset_deg)

    def calculate_Knee_Flex(self, thigh_quat, shank_quat):
        GB_thigh_q = thigh_quat * self.BS_q_thigh_inv
        GB_shank_q = shank_quat * self.BS_q_shank_inv
        q_rel = GB_thigh_q.inv() * GB_shank_q
        _, twist = get_swing_twist_decomposition(q_rel, FLEXION_AXIS_PARENT)
        return -signed_twist_angle_deg(twist, FLEXION_AXIS_PARENT)

    def calculate_Ankle_Flex(self, shank_quat, foot_quat):
        GB_shank_q = shank_quat * self.BS_q_shank_inv
        GB_foot_q = foot_quat * self.BS_q_foot_inv
        q_rel = GB_shank_q.inv() * GB_foot_q
        _, twist = get_swing_twist_decomposition(q_rel, FLEXION_AXIS_PARENT)
        return -signed_twist_angle_deg(twist, FLEXION_AXIS_PARENT)

    def calculate_Ankle_Inversion(self, shank_quat, foot_quat):
        GB_shank_q = shank_quat * self.BS_q_shank_inv
        GB_foot_q = foot_quat * self.BS_q_foot_inv

        e1 = _unit(GB_shank_q.apply(FLEXION_AXIS_PARENT))   # shank ML
        shank_long = _unit(GB_shank_q.apply(VERTICAL_AXIS)) # shank long
        foot_long = _unit(GB_foot_q.apply(VERTICAL_AXIS))   # foot long

        e2 = _normalize_or_none(np.cross(foot_long, e1))     # floating axis
        if e2 is None:
            return 0.0

        angle = signed_angle_about_axis(shank_long, foot_long, e2)
        return wrap_deg(-self.hip_side_sign * angle)

    def calculate_Hip_Adduction(self, pelvis_quat, thigh_quat):
        GB_pelvis_q = pelvis_quat * self.BS_q_pelvis_inv
        GB_thigh_q = thigh_quat * self.BS_q_thigh_inv

        e1 = _unit(GB_pelvis_q.apply(FLEXION_AXIS_PARENT))   # pelvis ML (flex axis)
        pelvis_long = _unit(GB_pelvis_q.apply(VERTICAL_AXIS))
        thigh_long = _unit(GB_thigh_q.apply(VERTICAL_AXIS))

        e2 = _normalize_or_none(np.cross(thigh_long, e1))    # floating axis
        if e2 is None:
            return 0.0

        angle = signed_angle_about_axis(pelvis_long, thigh_long, e2)
        return wrap_deg(self.hip_side_sign * angle)

    def calculate_Hip_Internal_Rotation(self, pelvis_quat, thigh_quat):
        GB_pelvis_q = pelvis_quat * self.BS_q_pelvis_inv
        GB_thigh_q = thigh_quat * self.BS_q_thigh_inv

        e1 = _unit(GB_pelvis_q.apply(FLEXION_AXIS_PARENT))   # pelvis ML
        pelvis_long = _unit(GB_pelvis_q.apply(VERTICAL_AXIS))
        thigh_long = _unit(GB_thigh_q.apply(VERTICAL_AXIS))
        thigh_ml = _unit(GB_thigh_q.apply(FLEXION_AXIS_PARENT))

        # Define AP axes from (long × ML). Direction may need a sign flip to match your conventions.
        pelvis_ap = _normalize_or_none(np.cross(pelvis_long, e1))
        thigh_ap = _normalize_or_none(np.cross(thigh_long, thigh_ml))
        if pelvis_ap is None or thigh_ap is None:
            return 0.0

        angle = signed_angle_about_axis(pelvis_ap, thigh_ap, thigh_long)
        return wrap_deg(self.hip_side_sign * angle)
