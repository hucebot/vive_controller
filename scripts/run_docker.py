#!/usr/bin/env python3
import argparse
import subprocess
import os
import sys

# Default Configuration
DEFAULT_REGISTRY = "registry.gitlab.inria.fr/eurobin-horizon/code/ros2-vive-controller"
IMAGE_NAME = "vive-controller"

# DEFAULT_REGISTRY = "registry.gitlab.com/bleurobotics/containers/vive_controller"
# IMAGE_NAME = "vive-controller-from-bleu-base-jazzy"
# CONTAINER_NAME = "ros2_vive_controller"

def get_project_version():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.dirname(script_dir)
    version_file = os.path.join(repo_root, "VERSION")
    try:
        with open(version_file, "r") as f:
            return f.read().strip()
    except FileNotFoundError:
        return "latest"

def run_docker(args):
    tag = args.tag if args.tag else get_project_version()
    if args.registry:
        full_image_name = f"{args.registry}/{IMAGE_NAME}:{tag}"
    else:
        full_image_name = f"{IMAGE_NAME}:{tag}"

    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.dirname(script_dir)

    # Mission Aliases
    missions = {
        "calibrate": "ros2 launch ros2_vive_controller calibration.launch.py",
        "teleop": "ros2 launch ros2_vive_controller vive_teleop.launch.py",
        "g1": "ros2 launch ros2_vive_controller g1_dual.launch.py",
        "tiago": "ros2 launch ros2_vive_controller tiago_dual.launch.py",
        "franka": "ros2 launch ros2_vive_controller franka_single.launch.py"
    }

    # Resolve command
    user_cmd_str = missions.get(args.command, args.command) if args.command else "/bin/bash"

    # 1. Check if container is already running
    is_running = subprocess.getoutput(f"docker ps -q -f name={CONTAINER_NAME}")
    if is_running:
        print(f"--- Container {CONTAINER_NAME} is already running. Joining... ---")
        exec_cmd = ["docker", "exec", "-it", CONTAINER_NAME]
        exec_cmd.append("/entrypoint.sh")
        exec_cmd.extend(user_cmd_str.split())
        subprocess.call(exec_cmd)
        return

    # 2. GUI Setup
    subprocess.call(["xhost", "+local:docker"], stdout=subprocess.DEVNULL)

    # Path logic
    config_dir = os.path.join(repo_root, "config")
    steam_cfg = os.path.join(config_dir, "steamvr_config")

    # 3. Construct Docker Run Command
    cmd = [
        "docker", "run", "-it", "--rm",
        "--name", CONTAINER_NAME,
        "--gpus", "all",
        "--net", "host",
        "--ipc", "host",
        "--pid", "host",
        "--privileged",
        # Environment
        "-e", f"DISPLAY={os.getenv('DISPLAY')}",
        "-e", "NVIDIA_DRIVER_CAPABILITIES=all",
        "-e", f"XDG_RUNTIME_DIR={os.getenv('XDG_RUNTIME_DIR')}",
        "-e", "QT_X11_NO_MITSHM=1",
        "-e", f"ROS_DOMAIN_ID={args.domain}",
        "-e", "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp",
        # "-e", "CYCLONEDDS_URI=/ros2_ws/src/ros2_vive_controller/config/cyclonedds.xml",
        # Hardware
        "-v", "/tmp/.X11-unix:/tmp/.X11-unix",
        "-v", "/dev:/dev",
        "-v", "/run/udev:/run/udev",
        # SteamVR
        "-v", f"{steam_cfg}/openvrpaths.vrpath:/root/.config/openvr/openvrpaths.vrpath",
        "-v", f"{steam_cfg}/default_driver_null:/home/steam/Steam/steamapps/common/SteamVR/drivers/null/resources/settings/default.vrsettings",
        "-v", f"{steam_cfg}/default_resources:/home/steam/Steam/steamapps/common/SteamVR/resources/settings/default.vrsettings",
        "-w", "/ros2_ws",
    ]

    # 4. Granular Mounting Strategy (Dev Mode)
    # Instead of mounting the whole root (which overwrites marker files), we mount only what we edit.
    if not args.prod:
        print(f"[INFO] Running in DEV mode (Granular Hot-Reload)")

        # Mount the Python Source Code (Changes here apply instantly)
        cmd.extend(["-v", f"{repo_root}/ros2_vive_controller:/ros2_ws/src/ros2_vive_controller/ros2_vive_controller"])

        # Mount Launch Files
        cmd.extend(["-v", f"{repo_root}/launch:/ros2_ws/src/ros2_vive_controller/launch"])

        # Mount Config Files
        cmd.extend(["-v", f"{repo_root}/config:/ros2_ws/src/ros2_vive_controller/config"])

        # Mount RViz Configs
        cmd.extend(["-v", f"{repo_root}/rviz:/ros2_ws/src/ros2_vive_controller/rviz"])

        # Mount Assets (Meshes/Images)
        cmd.extend(["-v", f"{repo_root}/assets:/ros2_ws/src/ros2_vive_controller/assets"])

        # Mount setup.py
        cmd.extend(["-v", f"{repo_root}/setup.py:/ros2_ws/src/ros2_vive_controller/setup.py"])

        # Mount Entrypoint
        cmd.extend(["-v", f"{repo_root}/.ci/entrypoint.sh:/entrypoint.sh"])

    else:
        print(f"[INFO] Running in PROD mode (Using baked-in image code)")

    # 5. Finalize
    cmd.append(full_image_name)
    cmd.append("/entrypoint.sh")
    cmd.extend(user_cmd_str.split())

    print(f"\n🚀 Launching {CONTAINER_NAME}...")
    print(f"   Domain ID: {args.domain}")
    print(f"   Command:   {user_cmd_str}\n")

    try:
        subprocess.check_call(cmd)
    except subprocess.CalledProcessError:
        pass

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run Vive Controller Container")
    parser.add_argument("command", nargs="?", help="Mission alias or raw command")
    parser.add_argument("--domain", type=int, default=1, help="ROS_DOMAIN_ID")
    parser.add_argument("--registry", default=DEFAULT_REGISTRY)
    parser.add_argument("--tag", help="Docker tag")
    parser.add_argument("--prod", action="store_true", help="Run in prod mode")

    args = parser.parse_args()
    run_docker(args)
