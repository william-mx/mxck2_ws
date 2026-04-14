import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration


# Usage examples:
# ros2 launch mxck_run export_images_launch.py filename:=traffic_sign_seq_vol_II
# ros2 launch mxck_run export_images_launch.py filename:=traffic_sign_seq_vol_II topic:=/camera/depth/image_raw


def setup_output_path(context, *args, **kwargs):
    """
    Creates output directory structure and generates appropriate path.
    Structure: <package>/export/<bagfile_name>/<topic_name>/frame_XXXX.jpg
    """
    filename_input = LaunchConfiguration("filename").perform(context)
    topic = LaunchConfiguration("topic").perform(context)
    custom_out = LaunchConfiguration("out_dir").perform(context)

    # Get package root
    package_root = Path("/mxck2_ws/src/mxck_run")

    # Resolve bag path - check if it's a full path or just a filename
    bag_path = Path(filename_input)
    if not bag_path.exists():
        potential_path = package_root / "bagfiles" / filename_input
        if potential_path.exists():
            bag_path = potential_path

    # If it's a directory, find the .mcap inside
    if bag_path.is_dir():
        mcap_files = list(bag_path.glob("*.mcap"))
        if not mcap_files:
            raise FileNotFoundError(f"No .mcap file found in {bag_path}")
        bag_path = mcap_files[0]
        print(f"Found mcap: {bag_path}")

    # Get bag filename without extension
    bag_name = Path(filename_input).name

    # Sanitize topic name for filesystem (remove leading /, replace / with _)
    topic_safe = topic.lstrip('/').replace('/', '_')

    # Create export directory structure
    if custom_out and custom_out != "":
        export_dir = package_root / "export" / bag_name / custom_out
    else:
        export_dir = package_root / "export" / bag_name / topic_safe

    export_dir.mkdir(parents=True, exist_ok=True)

    print(f"\n{'='*60}")
    print(f"Image Export Configuration:")
    print(f"  Bag file: {bag_path}")
    print(f"  Bag name: {bag_name}")
    print(f"  Topic: {topic}")
    print(f"  Output directory: {export_dir}")
    print(f"{'='*60}\n")

    # Remap the bag topic to a unique name so live camera frames are never captured
    bag_topic = "/bag" + topic  # e.g. /bag/camera/camera/color/image_raw

    # Extract images — listens only to the remapped bag topic
    extract_images = ExecuteProcess(
        cmd=[
            "bash", "-c",
            f"cd \"{export_dir}\" && "
            f"ros2 run image_view extract_images --ros-args -r image:={bag_topic}"
        ],
        output="screen"
    )

    # Play the bag — camera topics only (no ackermann_cmd), remapped to /bag/...
    play = ExecuteProcess(
        cmd=[
            "ros2", "bag", "play", str(bag_path),
            "--rate", "0.2",
            "--topics",
            "/camera/camera/color/image_raw",
            "/camera/camera/infra1/image_rect_raw",
            "/camera/camera/infra2/image_rect_raw",
            "/tf",
            "/tf_static",
            "--remap",
            f"{topic}:={bag_topic}"
        ],
        output="screen"
    )

    # Shut everything down when the bag finishes
    shutdown_on_bag_end = RegisterEventHandler(
        OnProcessExit(
            target_action=play,
            on_exit=[Shutdown()]
        )
    )

    return [extract_images, play, shutdown_on_bag_end]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "filename",
            description="Bag folder name (will check <package>/bagfiles/) or full path"
        ),
        DeclareLaunchArgument(
            "topic",
            default_value="/camera/camera/color/image_raw",
            description="Image topic to extract"
        ),
        DeclareLaunchArgument(
            "out_dir",
            default_value="",
            description="Custom output subdirectory name (optional, default: <topic_name>)"
        ),
        OpaqueFunction(function=setup_output_path)
    ])