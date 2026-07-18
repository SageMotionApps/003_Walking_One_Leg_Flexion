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
        is_1d = False

        # Reshape to 2D if input is 1D
        if quat.ndim == 1:
            quat = quat.reshape((1, -1))
            is_1d = True
        elif quat.ndim != 2:
            raise ValueError("Invalid quaternion shape. Should be 1D or 2D.")
        if quat.shape[1] != 4:
            raise ValueError(
                "Invalid quaternion shape. Should have exactly 4 elements."
            )
        if scalar_first:
            quat = np.roll(quat, -1, axis=1)
        rotation = super().from_quat(quat)

        # Return the original 1D form if input was 1D
        return rotation if not is_1d else rotation[0]

    def as_quat(self, canonical=False, scalar_first=False):
        quat = super().as_quat()
        if Version(scipy.__version__) >= Version("1.11.0"):
            quat = super().as_quat(canonical=canonical)

        is_1d = False

        # Reshape to 2D if input was 1D
        if quat.ndim == 1:
            quat = quat.reshape((1, -1))
            is_1d = True
        elif quat.ndim != 2:
            raise ValueError("Invalid quaternion shape. Should be 1D or 2D.")
        if quat.shape[1] != 4:
            raise ValueError(
                "Invalid quaternion shape. Should have exactly 4 elements."
            )
        # Adjust order if scalar_first is True
        if scalar_first:
            quat = np.roll(quat, 1, axis=1)

        # Return the original 1D form if input was 1D
        return quat[0] if is_1d else quat


def get_rotation_class():
    if Version(scipy.__version__) >= Version("1.14.0"):
        print("Using newer scipy version")
        return ScipyRotation
    else:
        print("Using older scipy version")
        return R_FIXED


Rotation = get_rotation_class()


class RotationScalarFirst(Rotation):
    error_message = "Hey, how's it going? \nI got a little question for you.\nWhy are you using scalar_first=False with RotationScalarFirst?\nAre you okay?\nDo you need someone to talk to? \nIs it a late night and you are trying to get something finished on time? \nI am going to let you do what you want to do, but maybe you should reconsider some of your life choices.\nI hope that you find peace and happiness in life if you continue to use scalar_first=False with RotationScalarFirst."

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def as_quat(self, canonical=False, scalar_first=True):
        if not scalar_first:
            print(self.error_message)
        quat = super().as_quat(canonical=canonical, scalar_first=scalar_first)
        return quat

    @classmethod
    def from_euler(cls, seq, angles, degrees=False):
        rotation = super().from_euler(seq, angles, degrees=degrees)
        return rotation if isinstance(rotation, cls) else cls(rotation.as_quat())

    @classmethod
    def from_quat(cls, quat, scalar_first=True):
        if not scalar_first:
            print(cls.error_message)
        rotation = super().from_quat(quat, scalar_first=scalar_first)
        return rotation if isinstance(rotation, cls) else cls(rotation.as_quat())


# To use this class, you can `from .Rotation import Rotation as R`.
# When scipy is greater than 1.14.0, this will use the newer version of scipy.
# This case is identical to `from scipy.spatial.transform import Rotation as R`.
# Otherwise, it will use the polyfill in this file that implements identical functionality.
#
# You can also use RotationScalarFirst, which is identical to Rotation but with scalar_first=True.
# This just changes the default behavior to be more intuitive for people who are used to using the
# scalar first convention.


def test_module():
    # Test 1D input and output
    R = R_FIXED.from_quat([1, 0, 0, 0], scalar_first=True)
    assert np.allclose(
        R.as_quat(scalar_first=True), [1, 0, 0, 0]
    ), "Test failed: 1D input with scalar_first=True."

    R = R_FIXED.from_quat([1, 0, 0, 0], scalar_first=False)
    assert np.allclose(
        R.as_quat(scalar_first=False), [1, 0, 0, 0]
    ), "Test failed: 1D input with scalar_first=False."

    # Test 2D input and output
    R = R_FIXED.from_quat([[1, 0, 0, 0]], scalar_first=True)
    assert np.allclose(
        R.as_quat(scalar_first=True), [[1, 0, 0, 0]]
    ), "Test failed: 2D input with scalar_first=True."

    R = R_FIXED.from_quat([[0, 0, 0, 1]], scalar_first=False)
    assert np.allclose(
        R.as_quat(scalar_first=False), [[0, 0, 0, 1]]
    ), "Test failed: 2D input with scalar_first=False."

    # Test 1D default
    R = R_FIXED.from_quat([0, 0, 0, 1])
    assert np.allclose(
        R.as_quat(), [0, 0, 0, 1]
    ), "Test failed: 1D input with default scalar order."

    # Test 2D default
    R = R_FIXED.from_quat([[0, 0, 0, 1]])
    assert np.allclose(
        R.as_quat(), [[0, 0, 0, 1]]
    ), "Test failed: 2D input with default scalar order."

    # Test Euler to quaternion conversion (1D input)
    R = R_FIXED.from_euler("zyx", [0, 0, 0], degrees=True)
    assert np.allclose(
        R.as_quat(scalar_first=True), [1, 0, 0, 0]
    ), "Test failed: Euler to quaternion conversion (1D)."

    # Test Euler to quaternion conversion (2D input)
    R = R_FIXED.from_euler("zyx", [[0, 0, 0]], degrees=True)
    assert np.allclose(
        R.as_quat(scalar_first=True), [[1, 0, 0, 0]]
    ), "Test failed: Euler to quaternion conversion (2D)."

    # Test scalar_first
    R = RotationScalarFirst.from_euler("zyx", [0, 0, 0], degrees=True)
    assert np.allclose(R.as_quat(), [1, 0, 0, 0]), "Test failed: scalar_first=True."

    print("All tests passed!")


test_module()
