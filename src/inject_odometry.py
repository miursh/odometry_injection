import argparse
import os.path as osp
from pathlib import Path
import shutil

from localizer.localizer import localize
from localizer.ros2_bag_utils import RosbagHandler
from localizer.sensor_data import lift_from_3d_to_6d


def resolve_bag_uri(path: str) -> str:
    """Accept a rosbag directory or a single .mcap/.db3 file and return the URI to open."""
    p = Path(path)
    if p.is_dir():
        return str(p)
    if p.is_file() and p.suffix in {".mcap", ".db3"}:
        return str(p)
    raise ValueError(f"Unsupported bag path: {path}")


def ensure_writable_bag_dir(path: str, overwrite: bool = False) -> str:
    """
    Return a directory path that does not exist yet for rosbag2 writer.

    - If path exists and overwrite=True, delete it and return the same path.
    - If path exists and overwrite=False, return a new unique path with suffix _1, _2, ...
    - Do not create the directory here; rosbag2 will create it when opening the writer.
    """
    p = Path(path)
    parent = p.parent
    parent.mkdir(parents=True, exist_ok=True)

    if p.exists():
        if overwrite:
            shutil.rmtree(p)
            return str(p)
        # choose a unique path
        base = str(p)
        idx = 1
        while True:
            candidate = Path(f"{base}_{idx}")
            if not candidate.exists():
                return str(candidate)
            idx += 1
    return str(p)


def main() -> int:
    parser = argparse.ArgumentParser(description="Inject localization topics into a single rosbag if missing")
    parser.add_argument("-i", "--input", required=True, help="Path to rosbag directory or data file (.mcap/.db3)")
    parser.add_argument(
        "-o", "--output", default=None, help="Output directory to write new bag. Defaults to <bag_dir>_loc"
    )
    parser.add_argument("--localization-topic", default="/odometry", help="Odometry topic name to write")
    parser.add_argument("--yaw-rate-topic", default="/sensing/ins/oxts/imu", help="Yaw rate topic name to read")
    parser.add_argument(
        "--velocity-topic",
        default="/vehicle/status/velocity_status",
        help="Velocity topic name to read. Either VelocitiyReport or TwistStamped",
    )
    parser.add_argument(
        "--velocity-ratio", type=float, default=1.0, help="Scale factor to apply to the velocity readings"
    )
    parser.add_argument("--overwrite", action="store_true", help="If output dir exists, delete and overwrite it")
    args = parser.parse_args()

    bag_uri = resolve_bag_uri(args.input)
    bag_uri_path = Path(bag_uri)
    bag_name = bag_uri_path.stem if bag_uri_path.is_file() else bag_uri_path.name
    desired_out_dir = args.output or osp.join(str(bag_uri_path.parent), f"{bag_name}_loc")
    out_dir = ensure_writable_bag_dir(desired_out_dir, overwrite=args.overwrite)

    handler = RosbagHandler(
        bag_uri,
        out_dir,
        localization_topic=args.localization_topic,
        yaw_rate_topic=args.yaw_rate_topic,
        velocity_topic=args.velocity_topic,
        velocity_ratio=args.velocity_ratio,
    )

    status = handler.localization_topic_status
    print(f"Localization topic status: {status}")
    if status == "ONBOARD":
        print("Bag already has valid localization; copying other topics only and exiting.")
        # When ONBOARD, we simply exit; no injection needed.
        return 0

    if not handler.can_localize:
        print("IMU and/or velocity topics missing – cannot localize.")
        return 2

    # Run dead-reckoning localization
    poses3d = localize(handler.yaw_rate_sequence, handler.velocity_sequence)
    poses6d = lift_from_3d_to_6d(poses3d)
    handler.update_localization_topics(poses6d)
    print(f"Wrote injected bag to: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
