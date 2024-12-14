import scipy
from packaging.version import Version
from scipy.spatial.transform import Rotation as ScipyRotation
import numpy as np


# The scipy Rotations library is great, but it does not allow you to
# specify the order of the quaternion components, and it defaults to xyzw.
# This overwrites the scipy Rotation class to allow for this.
class R_FIXED(ScipyRotation):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    @classmethod
    def from_quat(cls, quat, scalar_first=False):
        quat = np.asarray(quat)
        if quat.ndim == 1:
            quat = quat.reshape((1, -1))
        elif quat.ndim != 2:
            raise ValueError("Invalid quaternion shape. Should be 1D or 2D.")
        if quat.shape[1] != 4:
            raise ValueError(
                "Invalid quaternion shape. Should have exactly 4 elements."
            )
        if scalar_first:
            return super().from_quat(np.roll(quat, -1, axis=1))
        return super().from_quat(quat)

    def as_quat(self, canonical=False, scalar_first=False):
        if Version(scipy.__version__) >= Version("1.11.0"):
            quat = super().as_quat(canonical=canonical)
        else:
            quat = super().as_quat()
        if quat.ndim == 1:
            quat = quat.reshape((1, -1))
        elif quat.ndim != 2:
            raise ValueError("Invalid quaternion shape. Should be 1D or 2D.")
        if quat.shape[1] != 4:
            raise ValueError(
                "Invalid quaternion shape. Should have exactly 4 elements."
            )
        # Get the quaternion in 'xyzw' order
        if scalar_first:
            return np.roll(quat, 1, axis=1)
        return quat


def test_module():
    R = R_FIXED.from_quat([1, 0, 0, 0], scalar_first=True)
    assert np.allclose(
        R.as_quat(scalar_first=True), [1, 0, 0, 0]
    ), "One of the tests for the R_FIXED class failed."

    R = R_FIXED.from_quat([1, 0, 0, 0], scalar_first=False)
    assert np.allclose(
        R.as_quat(scalar_first=False), [1, 0, 0, 0]
    ), "One of the tests for the R_FIXED class failed."

    R = R_FIXED.from_quat([0, 0, 0, 1])
    assert np.allclose(
        R.as_quat(), [0, 0, 0, 1]
    ), "One of the tests for the R_FIXED class failed."

    R = R_FIXED.from_euler("zyx", [0, 0, 0], degrees=True)
    assert np.allclose(
        R.as_quat(scalar_first=True), [1, 0, 0, 0]
    ), "One of the tests for the R_FIXED class failed."


test_module()


def get_rotation_class():
    if Version(scipy.__version__) >= Version("1.14.0"):
        print("Using newer scipy version")
        return ScipyRotation
    else:
        print("Using older scipy version")
        return R_FIXED


Rotation = get_rotation_class()

# To use this class, you can `from .Rotation import Rotation as R`.
# When scipy is greater than 1.14.0, this will use the newer version of scipy.
# This case is identical to `from scipy.spatial.transform import Rotation as R`.
# Otherwise, it will use the polyfill in this file that implements identical functionality.
