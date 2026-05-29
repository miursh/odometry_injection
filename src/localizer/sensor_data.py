from bisect import bisect_left
import math


class YawRateData:
    def __init__(self, stamp: float, yaw_rate: float) -> None:
        self.stamp = stamp
        self.yaw_rate = yaw_rate


class YawRateSequence:
    def __init__(self) -> None:
        self.data_sequence: list[YawRateData] = []
        self.timestamp_sequence: list[float] = []

    def append(self, data: YawRateData) -> None:
        # check if the timestamp is in ascending order
        if len(self.data_sequence) > 0 and self.data_sequence[-1].stamp > data.stamp:
            return
        self.data_sequence.append(data)
        self.timestamp_sequence.append(data.stamp)

    def interpolate(
        self, stamp: float, window_size: int = 11, method: str = "gaussian", sigma: float | None = None
    ) -> YawRateData:
        """
        Interpolate (and lightly smooth) the yaw rate at the given timestamp.

        - Uses a local window around the target timestamp.
        - Default weighting is Gaussian in time to reduce high-frequency noise.
        - Falls back to nearest neighbor if weights degenerate.
        """
        pos = bisect_left(self.timestamp_sequence, stamp)
        # If exact timestamp exists, return it directly
        if pos < len(self.timestamp_sequence) and self.timestamp_sequence[pos] == stamp:
            return YawRateData(stamp=stamp, yaw_rate=self.data_sequence[pos].yaw_rate)
        window_half_size = window_size // 2

        # Handle boundary conditions
        if pos < window_half_size:
            start = 0
            end = min(window_size, len(self.timestamp_sequence))
        elif pos + window_half_size >= len(self.timestamp_sequence):
            start = max(0, len(self.timestamp_sequence) - window_size)
            end = len(self.timestamp_sequence)
        else:
            start = pos - window_half_size
            end = pos + window_half_size + 1

        # Estimate a reasonable sigma from local sampling if not provided
        if sigma is None:
            if end - start >= 2:
                dt_total = self.timestamp_sequence[end - 1] - self.timestamp_sequence[start]
                avg_dt = dt_total / max(1, (end - start - 1))
            else:
                avg_dt = 0.02  # assume ~50Hz as a safe default
            sigma = max(1e-3, 2.0 * avg_dt)

        # Perform weighted interpolation
        total_weight = 0.0
        weighted_sum = 0.0
        for i in range(start, end):
            t = self.timestamp_sequence[i]
            wz = self.data_sequence[i].yaw_rate
            dt = stamp - t
            weight = math.exp(-(dt * dt) / (2.0 * sigma * sigma)) if method == "gaussian" else 1.0 / (abs(dt) + 1e-9)
            total_weight += weight
            weighted_sum += wz * weight

        if total_weight <= 1e-12:
            # Fallback to nearest neighbor
            nn_idx = min(max(0, pos - 1), len(self.timestamp_sequence) - 1)
            return YawRateData(stamp=stamp, yaw_rate=self.data_sequence[nn_idx].yaw_rate)

        interpolated_yaw_rate = weighted_sum / total_weight

        return YawRateData(stamp=stamp, yaw_rate=interpolated_yaw_rate)


class VelocityData:
    def __init__(self, stamp: float, velocity: float) -> None:
        self.stamp = stamp
        self.velocity = velocity


class VelocitySequence:
    def __init__(self) -> None:
        self.data_sequence: list[VelocityData] = []
        self.timestamp_sequence: list[float] = []

    def append(self, data: VelocityData) -> None:
        # check if the timestamp is in ascending order
        if len(self.data_sequence) > 0 and self.data_sequence[-1].stamp > data.stamp:
            return
        self.data_sequence.append(data)
        self.timestamp_sequence.append(data.stamp)

    def interpolate(self, stamp: float) -> VelocityData:
        """Interpolate the velocity at the given timestamp with binary search."""
        pos = bisect_left(self.timestamp_sequence, stamp)
        if pos == 0:
            return VelocityData(stamp=stamp, velocity=self.data_sequence[0].velocity)
        if pos == len(self.timestamp_sequence):
            return VelocityData(stamp=stamp, velocity=self.data_sequence[-1].velocity)
        t0 = self.timestamp_sequence[pos - 1]
        t1 = self.timestamp_sequence[pos]
        v0 = self.data_sequence[pos - 1].velocity
        v1 = self.data_sequence[pos].velocity

        return VelocityData(stamp=stamp, velocity=(v1 - v0) / (t1 - t0) * (stamp - t0) + v0)


class PoseData6D:
    def __init__(
        self, stamp: float, px: float, py: float, pz: float, qx: float, qy: float, qz: float, qw: float
    ) -> None:
        self.stamp = stamp
        self.px = px
        self.py = py
        self.pz = pz
        self.qx = qx
        self.qy = qy
        self.qz = qz
        self.qw = qw


class PoseSequence6D:
    def __init__(self) -> None:
        self.data_sequence: list[PoseData6D] = []

    def append(self, data: PoseData6D) -> None:
        self.data_sequence.append(data)


class PoseData3D:
    def __init__(self, stamp: float, px: float, py: float, yaw: float) -> None:
        self.stamp = stamp
        self.px = px
        self.py = py
        self.yaw = yaw


class PoseSequence3D:
    def __init__(self) -> None:
        self.data_sequence: list[PoseData3D] = []

    def append(self, data: PoseData3D) -> None:
        self.data_sequence.append(data)


def lift_from_3d_to_6d(pose_sequence_3d: PoseSequence3D) -> PoseSequence6D:
    pose_sequence_6d = PoseSequence6D()

    for pose_3d in pose_sequence_3d.data_sequence:
        px = pose_3d.px
        py = pose_3d.py
        pz = 0.0  # Since it's a 2D pose lifted to 6D, we set z to 0
        yaw = pose_3d.yaw

        # Correct quaternion calculation
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw / 2)
        qw = math.cos(yaw / 2)

        pose_6d = PoseData6D(
            stamp=pose_3d.stamp,
            px=px,
            py=py,
            pz=pz,
            qx=qx,
            qy=qy,
            qz=qz,
            qw=qw,
        )
        pose_sequence_6d.append(pose_6d)

    return pose_sequence_6d
