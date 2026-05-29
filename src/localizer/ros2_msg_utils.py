from builtin_interfaces.msg import Time
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu

try:
    from vehicle_msgs.msg import VelocityReport
except ImportError:
    VelocityReport = None

from .sensor_data import VelocityData
from .sensor_data import YawRateData


def get_yaw_rate_data_from_imu(imu: Imu | TwistStamped) -> YawRateData:
    if isinstance(imu, TwistStamped):
        return YawRateData(
            stamp=stamp_to_unix_timestamp(imu.header.stamp),
            yaw_rate=imu.twist.angular.z,
        )
    elif isinstance(imu, Imu):
        return YawRateData(
            stamp=stamp_to_unix_timestamp(imu.header.stamp),
            yaw_rate=imu.angular_velocity.z,
        )
    else:
        err_msg = f"Unsupported IMU type: {type(imu)}"
        raise TypeError(err_msg)


def get_velocity_data(velocity_report: object, velocity_ratio: float) -> VelocityData:
    if isinstance(velocity_report, TwistStamped):
        return VelocityData(
            stamp=stamp_to_unix_timestamp(velocity_report.header.stamp),
            velocity=velocity_report.twist.linear.x * velocity_ratio,
        )
    elif VelocityReport is not None and isinstance(velocity_report, VelocityReport):
        return VelocityData(
            stamp=stamp_to_unix_timestamp(velocity_report.header.stamp),
            velocity=velocity_report.longitudinal_velocity * velocity_ratio,
        )
    else:
        err_msg = f"Unsupported velocity report type: {type(velocity_report)}"
        if VelocityReport is None:
            err_msg += " (vehicle_msgs is not installed in this ROS environment)"
        raise TypeError(err_msg)


def stamp_to_unix_timestamp(stamp: Time) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def unix_timestamp_to_stamp(timestamp: float) -> Time:
    sec_int = int(timestamp)
    nano_sec_int = (timestamp - sec_int) * 1e9
    return Time(seconds=sec_int, nanoseconds=nano_sec_int).to_msg()
