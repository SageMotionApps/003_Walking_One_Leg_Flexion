# This uses the Rotations.py file since it compensates for scalar_first. Make sure to get that file too
# if you use this.
from collections.abc import MutableMapping
from typing import Optional

from .Rotation import Rotation as R


class YawCorrection:
    def __init__(
        self,
        data: list[MutableMapping[str, float]],
        base_imu: int,
        starting_yaw_offsets: list[float],
    ) -> None:
        """
        This class takes in the data from the base IMU and the starting yaw offsets for each sensor.
        It then calculates the yaw error for each sensor and applies the correction to the quaternions.
        The correction is applied to the quaternions using the `correct_yaw` method.
        :param data: The data from the base IMU and all other sensors.
        :param base_imu: The index of the base IMU in the data.
        :param starting_yaw_offsets: A list of starting yaw offsets for each sensor.

        Example usage:
        ```
        data = self.my_sage.get_next_data()
        if self.iteration == 0:
            self.yaw_correction = YawCorrection(data, self.base_imu, [0, -90])
            # Base IMU is the imu that you want to use as the base for the yaw correction.
            # The starting yaw offsets are the yaw offsets you want to apply to the other
            # sensors if you know what yaw offset they should have. Make sure that the base
            # IMU has a yaw offset of 0 and the other sensors have the correct yaw offsets.

            # Do other initialization here

        data = self.yaw_correction.correct_yaw(data)
        ```
        """
        self.base_imu: int = base_imu
        self.yaw_transforms: list[Optional[R]] = [None] * len(data)
        assert len(data) == len(starting_yaw_offsets), (
            "Number of starting yaw offsets must match number of sensors"
        )

        # Compute base yaw
        quat_base = [data[base_imu][f"Quat{i + 1}"] for i in range(4)]
        R_quat_base = R.from_quat(quat_base, scalar_first=True)
        base_yaw, _, _ = R_quat_base.as_euler("ZYX", degrees=True)
        base_yaw = (base_yaw + 180) % 360 - 180

        # Global transform to rotate the whole set so base yaw -> 0
        self.global_yaw_transform: R = R.from_euler("Z", -base_yaw, degrees=True)

        # Base IMU gets only the global transform (so it ends up at yaw = 0)
        self.yaw_transforms[base_imu] = self.global_yaw_transform

        # Other sensors: first align to base yaw + desired offset, then apply global de-yaw
        for sensor_idx, _ in enumerate(data):
            if sensor_idx == base_imu:
                continue
            quat = [data[sensor_idx][f"Quat{i + 1}"] for i in range(4)]
            R_quat = R.from_quat(quat, scalar_first=True)
            yaw, _, _ = R_quat.as_euler("ZYX", degrees=True)
            yaw = (yaw + 180) % 360 - 180

            yaw_error = yaw - base_yaw - starting_yaw_offsets[sensor_idx]
            yaw_error = (yaw_error + 180) % 360 - 180
            print(f"Yaw error for {sensor_idx}: {yaw_error}")

            per_sensor = R.from_euler("Z", -yaw_error, degrees=True)
            # Apply per-sensor correction, then global base de-yaw so base ends up at zero
            self.yaw_transforms[sensor_idx] = self.global_yaw_transform * per_sensor

    def correct_yaw(
        self, data: list[MutableMapping[str, float]]
    ) -> list[MutableMapping[str, float]]:
        for sensor_idx, sensor in enumerate(data):
            # Apply transform to *all* sensors, including the base
            quat = [sensor[f"Quat{i + 1}"] for i in range(4)]
            R_quat = R.from_quat(quat, scalar_first=True)
            corrected_quat = self.yaw_transforms[sensor_idx] * R_quat
            corrected_quat = corrected_quat.as_quat(scalar_first=True)
            for i in range(4):
                data[sensor_idx][f"Quat{i + 1}"] = corrected_quat[i]
        return data