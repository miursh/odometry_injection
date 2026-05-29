import math

from .sensor_data import PoseData3D
from .sensor_data import PoseSequence3D
from .sensor_data import VelocityData
from .sensor_data import VelocitySequence
from .sensor_data import YawRateData
from .sensor_data import YawRateSequence


def localize(
    yaw_rate_sequence: YawRateSequence,
    velocity_sequence: VelocitySequence,
    localization_hz: float = 50.0,
    stop_velocity_threshold: float = 0.1,
) -> PoseSequence3D:
    """Localize based on yaw rate and velocity for a certain time range."""
    time_from: float = min(yaw_rate_sequence.timestamp_sequence[0], velocity_sequence.timestamp_sequence[0])
    time_to: float = max(yaw_rate_sequence.timestamp_sequence[-1], velocity_sequence.timestamp_sequence[-1])

    # Initialize the pose sequence
    pose_sequence: PoseSequence3D = PoseSequence3D()
    current_pose_3d: PoseData3D = PoseData3D(time_from, 0.0, 0.0, 0.0)
    while current_pose_3d.stamp < time_to:
        current_pose_3d.stamp += 1.0 / localization_hz

        yaw_rate_data: YawRateData = yaw_rate_sequence.interpolate(current_pose_3d.stamp)
        velocity_data: VelocityData = velocity_sequence.interpolate(current_pose_3d.stamp)

        # [IMPORTANT] Stop the vehicle if the velocity is below the threshold
        if abs(velocity_data.velocity) < stop_velocity_threshold:
            yaw_rate_data.yaw_rate = 0.0
            velocity_data.velocity = 0.0

        # Update the pose
        current_pose_3d.yaw += -yaw_rate_data.yaw_rate / localization_hz
        current_pose_3d.px += velocity_data.velocity / localization_hz * math.cos(current_pose_3d.yaw)
        current_pose_3d.py += velocity_data.velocity / localization_hz * math.sin(current_pose_3d.yaw)

        # Append the pose to the sequence
        pose_sequence.append(
            PoseData3D(
                stamp=current_pose_3d.stamp,
                px=current_pose_3d.px,
                py=current_pose_3d.py,
                yaw=current_pose_3d.yaw,
            )
        )

    return pose_sequence
