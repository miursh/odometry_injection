from collections.abc import Generator
import os
from pathlib import Path

import builtin_interfaces.msg
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message
from rclpy.time import Time
import rosbag2_py
from rosbag2_py import ConverterOptions
from rosbag2_py import SequentialReader
from rosbag2_py import SequentialWriter
from rosbag2_py import StorageFilter
from rosbag2_py import StorageOptions
from rosbag2_py import TopicMetadata
from rosidl_runtime_py.utilities import get_message
from tf2_msgs.msg import TFMessage
import tf_transformations as tf
from tqdm import tqdm
import yaml

from .ros2_msg_utils import get_velocity_data
from .ros2_msg_utils import get_yaw_rate_data_from_imu
from .sensor_data import PoseSequence6D
from .sensor_data import VelocitySequence
from .sensor_data import YawRateSequence


def get_default_converter_options() -> ConverterOptions:
    return ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )


def infer_storage_id(bag_dir: str) -> str:
    storage_ids = {".db3": "sqlite3", ".mcap": "mcap"}
    bag_dir_path = Path(bag_dir)
    if bag_dir_path.is_file():
        data_file = bag_dir_path
    else:
        data_file = next(p for p in bag_dir_path.glob("*") if p.suffix in storage_ids)
    if data_file.suffix not in storage_ids:
        raise ValueError(f"Unsupported storage id: {data_file.suffix}")
    return storage_ids[data_file.suffix]


def get_default_storage_options(bag_dir: str) -> StorageOptions:
    storage_id = infer_storage_id(bag_dir)
    return StorageOptions(uri=bag_dir, storage_id=storage_id)


def get_message_count(bag_uri: str) -> int | None:
    bag_path = Path(bag_uri)
    metadata_path = bag_path / "metadata.yaml" if bag_path.is_dir() else bag_path.with_name("metadata.yaml")
    if not metadata_path.exists():
        return None

    with metadata_path.open() as f:
        return yaml.safe_load(f)["rosbag2_bagfile_information"]["message_count"]


def get_options(
    bag_dir: str,
    storage_options: StorageOptions | None = None,
    converter_options: ConverterOptions | None = None,
) -> tuple[StorageOptions, ConverterOptions]:
    storage_options = storage_options if storage_options else get_default_storage_options(bag_dir)
    converter_options = converter_options if converter_options else get_default_converter_options()
    return storage_options, converter_options


def create_writer(bag_dir: str, storage_id: str) -> SequentialWriter:
    storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    writer = rosbag2_py.SequentialWriter()
    writer.open(storage_options, converter_options)
    return writer, storage_options, converter_options


def create_reader(
    bag_dir: str,
    storage_options: StorageOptions | None = None,
    converter_options: ConverterOptions | None = None,
) -> SequentialReader:
    storage_options, converter_options = get_options(bag_dir, storage_options, converter_options)
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    return reader


def get_topic_type_dict(bag_dir: str) -> dict[str, str]:
    reader = create_reader(bag_dir)

    topic_name_to_topic_type: dict[str, str] = {}
    for topic in reader.get_all_topics_and_types():
        topic_name_to_topic_type[topic.name] = topic.type

    return topic_name_to_topic_type


class RosbagHandler:
    DIAGNOSTICS_TOPIC = "/diagnostics"
    DIAGNOSTICS_TYPE = "diagnostic_msgs/msg/DiagnosticArray"
    DIAGNOSTICS_LOCALIZATION_STATUS_ID = [
        "localization_error_monitor: ellipse_error_status",
        "localization: localization_error_monitor",
        "localization_error_monitor: localization_accuracy",
        "localization_error_monitor: localization_accuracy_lateral_direction",
    ]

    # Threshold to determine if the rosbag has reliable localization information.
    # If the ratio of /diagnostics with message==OK is greater than this threshold for
    # DIAGNOSTICS_LOCALIZATION_STATUS_ID, the rosbag is deemed OK.
    LOCALIZATION_THRESHOLD_ONBOARD = 0.99

    def __init__(
        self,
        rosbag_dir: str,
        output_dir: str,
        localization_topic: str = "/localization/kinematic_state",
        yaw_rate_topic: str = "/ins/oxts/imu",
        velocity_topic: str = "/vehicle/status/velocity_status",
        velocity_ratio: float = 1.0,
    ) -> None:
        # parameters
        self._rosbag_dir = rosbag_dir
        self._output_dir = output_dir
        self._localization_topic = localization_topic
        # load rosbag
        self._topic_name_to_topic_type = get_topic_type_dict(self._rosbag_dir)

        self._yaw_rate_topic = yaw_rate_topic
        self._velocity_topic = velocity_topic
        self._velocity_ratio = velocity_ratio

        self.can_localize = (self._yaw_rate_topic in self._topic_name_to_topic_type) and (
            self._velocity_topic in self._topic_name_to_topic_type
        )

        # initialize member variables
        self._yaw_rate_sequence: YawRateSequence = YawRateSequence()
        self._velocity_sequence: VelocitySequence = VelocitySequence()

        # load topics from rosbag
        if self.can_localize:
            self._load_imu_and_velocity_topics()

    @property
    def localization_topic_status(self) -> str:
        """Returns the status of reliability of localization information included in the rosbag."""
        reader = create_reader(str(self._rosbag_dir))
        localization_diagnostics = []
        while reader.has_next():
            topic_name, data, timestamp = reader.read_next()
            if topic_name == RosbagHandler.DIAGNOSTICS_TOPIC:
                msg_type = get_message(RosbagHandler.DIAGNOSTICS_TYPE)
                msg = deserialize_message(data, msg_type)
                res = None
                for stat in msg.status:
                    if stat.name in RosbagHandler.DIAGNOSTICS_LOCALIZATION_STATUS_ID:
                        res = int.from_bytes(stat.level, byteorder="big")
                if res is not None:
                    localization_diagnostics.append(res == 0)

        if len(localization_diagnostics):
            localization_ok_ratio = sum(localization_diagnostics) / len(localization_diagnostics)
        else:
            localization_ok_ratio = 0

        if localization_ok_ratio > RosbagHandler.LOCALIZATION_THRESHOLD_ONBOARD:
            print(f"Localization ratio was {localization_ok_ratio} , no need localization injection")
            return "ONBOARD"
        else:
            print(f"Localization ratio was {localization_ok_ratio} , Need localization injection")
            return "NONE"

    @property
    def yaw_rate_sequence(self) -> YawRateSequence:
        return self._yaw_rate_sequence

    @property
    def velocity_sequence(self) -> VelocitySequence:
        return self._velocity_sequence

    def _load_imu_and_velocity_topics(self) -> None:
        assert self.can_localize, "missing topics for localization"

        for msg in self._read_messages([self._yaw_rate_topic]):
            self._yaw_rate_sequence.append(get_yaw_rate_data_from_imu(msg))
        for msg in self._read_messages([self._velocity_topic]):
            self._velocity_sequence.append(get_velocity_data(msg, self._velocity_ratio))

    def update_localization_topics(self, pose_sequence: PoseSequence6D) -> None:
        assert self.can_localize, "missing topics for localization"

        # Open the rosbag for writing
        storage_id = infer_storage_id(self._rosbag_dir)
        reader = create_reader(self._rosbag_dir)
        writer, storage_options, _ = create_writer(self._output_dir, storage_id)

        topic_type_dict: dict[str, str] = {}
        for topic_type in reader.get_all_topics_and_types():
            if topic_type.name in {"/tf", self._localization_topic}:
                continue
            topic_type_dict[topic_type.name] = topic_type.type
            writer.create_topic(topic_type)
        msg_num = get_message_count(self._rosbag_dir)
        with tqdm(total=msg_num) as pbar:
            while reader.has_next():
                pbar.update(1)
                topic_name, msg_bytes, stamp = reader.read_next()
                if topic_name in {"/tf", self._localization_topic}:
                    continue
                writer.write(topic_name, msg_bytes, stamp)
        # reindex to update metadata.yaml
        topic_metadata = TopicMetadata(
            name=self._localization_topic, type="nav_msgs/msg/Odometry", serialization_format="cdr"
        )
        writer.create_topic(topic_metadata)

        tf_topic_metadata = TopicMetadata(name="/tf", type="tf2_msgs/msg/TFMessage", serialization_format="cdr")
        writer.create_topic(tf_topic_metadata)

        # Initialize the first pose as the reference
        reference_pose = pose_sequence.data_sequence[0]
        reference_translation = [reference_pose.px, reference_pose.py, reference_pose.pz]
        reference_rotation = [
            reference_pose.qx,
            reference_pose.qy,
            reference_pose.qz,
            reference_pose.qw,
        ]

        for pose in pose_sequence.data_sequence:
            # Create the Odometry message
            msg = Odometry()
            timestamp = Time(seconds=pose.stamp)
            msg.header.stamp = timestamp.to_msg()
            msg.header.frame_id = "map"
            msg.child_frame_id = "base_link"
            msg.pose.pose.position.x = pose.px
            msg.pose.pose.position.y = pose.py
            msg.pose.pose.position.z = pose.pz
            msg.pose.pose.orientation.x = pose.qx
            msg.pose.pose.orientation.y = pose.qy
            msg.pose.pose.orientation.z = pose.qz
            msg.pose.pose.orientation.w = pose.qw

            # Serialize and write the Odometry message
            writer.write(self._localization_topic, serialize_message(msg), timestamp.nanoseconds)

            # Calculate the relative transform
            current_translation = [pose.px, pose.py, pose.pz]
            current_rotation = [pose.qx, pose.qy, pose.qz, pose.qw]

            # Convert to transformation matrices
            ref_matrix = tf.compose_matrix(
                translate=reference_translation,
                angles=tf.euler_from_quaternion(reference_rotation),
            )
            cur_matrix = tf.compose_matrix(
                translate=current_translation, angles=tf.euler_from_quaternion(current_rotation)
            )

            # Compute relative transformation
            relative_matrix = tf.concatenate_matrices(tf.inverse_matrix(ref_matrix), cur_matrix)
            relative_translation = tf.translation_from_matrix(relative_matrix)
            relative_rotation = tf.quaternion_from_matrix(relative_matrix)

            # Create the TransformStamped message
            transform = TransformStamped()
            transform.header.stamp = timestamp.to_msg()
            transform.header.frame_id = "map"
            transform.child_frame_id = "base_link"
            transform.transform.translation.x = relative_translation[0]
            transform.transform.translation.y = relative_translation[1]
            transform.transform.translation.z = relative_translation[2]
            transform.transform.rotation.x = relative_rotation[0]
            transform.transform.rotation.y = relative_rotation[1]
            transform.transform.rotation.z = relative_rotation[2]
            transform.transform.rotation.w = relative_rotation[3]

            # Create the TFMessage
            tf_msg = TFMessage(transforms=[transform])

            # Serialize and write the TF message
            writer.write("/tf", serialize_message(tf_msg), timestamp.nanoseconds)

        # Reindex to update metadata.yaml
        rosbag2_py.Reindexer().reindex(storage_options)
        del writer

    def _read_messages(self, topics: list[str], start_time: builtin_interfaces.msg.Time = None) -> Generator:
        assert self.can_localize, "missing topics for localization"

        if start_time is not None:
            start_time = Time.from_msg(start_time)

        reader = create_reader(self._rosbag_dir)
        if len(topics) != 0:
            reader.set_filter(StorageFilter(topics=topics))

        while reader.has_next():
            topic_name, data, timestamp = reader.read_next()
            topic_type = self._topic_name_to_topic_type[topic_name]

            # fails to deserialize Marker messages
            # https://docs.ros.org/en/rolling/Releases/Release-Humble-Hawksbill.html#support-textures-and-embedded-meshes-for-marker-messages
            if topic_type.startswith("visualization_msgs"):
                continue

            message = deserialize_message(data, get_message(topic_type))

            if start_time is not None:
                if hasattr(message, "header"):
                    message_time = Time.from_msg(message.header.stamp)
                elif hasattr(message, "stamp"):
                    message_time = Time.from_msg(message.stamp)
                else:
                    raise AttributeError(f"message has no time attribute: {type(message)}")

                if message_time < start_time:
                    continue

            yield message

    def input_bag_has_topic(self, topic: str) -> bool:
        """
        Check if a topic exists in the rosbag.

        :param topic_name: Name of the topic to check
        :return: True if the topic exists, False otherwise
        """
        return topic in self._topic_name_to_topic_type
