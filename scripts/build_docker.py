#!/usr/bin/env python3
import argparse
import subprocess
import os
import sys

# Default Configuration
DEFAULT_REGISTRY = "registry.gitlab.inria.fr/eurobin-horizon/code/vive_controller_tiago"
IMAGE_NAME = "vive-controller"

def get_project_version():
    """Reads the version from the VERSION file in the repo root."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.dirname(script_dir)
    version_file = os.path.join(repo_root, "VERSION")

    try:
        with open(version_file, "r") as f:
            return f.read().strip()
    except FileNotFoundError:
        print("WARNING: VERSION file not found. Defaulting to 'latest'.")
        return "latest"

def build_docker(args):
    # Determine Tag
    tag = args.tag if args.tag else get_project_version()

    # Construct full image name
    full_image_name = f"{args.registry}/{IMAGE_NAME}:{tag}"

    # Calculate Paths
    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.dirname(script_dir)
    # Correct path to Dockerfile inside .ci/
    dockerfile_path = os.path.join(repo_root, ".ci", "Dockerfile")

    print(f"\n[BUILD] Registry:     {args.registry}")
    print(f"[BUILD] Target Stage: {args.target}")
    print(f"[BUILD] Image Tag:    {full_image_name}")
    print(f"[BUILD] Dockerfile:   {dockerfile_path}")
    print(f"[BUILD] Context:      {repo_root}\n")

    # Build Command
    cmd = [
        "docker", "build",
        "-t", full_image_name,
        "-f", dockerfile_path,
        "--target", args.target,
    ]

    # --- NEW: Add No-Cache Flag ---
    if args.no_cache:
        print("[INFO] Building with --no-cache (Forcing rebuild)")
        cmd.append("--no-cache")
    # ------------------------------

    # Add Steam Credentials if building stages that descend from 'dep'
    # 'dep', 'dev', and 'app' all require these for the SteamVR installation step
    if args.target in ["dep", "dev", "app"]:
        steam_user = os.getenv("STEAM_USER")
        steam_pass = os.getenv("STEAM_PASSWORD")

        if not steam_user or not steam_pass:
            print("❌ Error: STEAM_USER and STEAM_PASSWORD environment variables must be set.")
            print("   Run: export STEAM_USER='your_user' STEAM_PASSWORD='your_password'")
            sys.exit(1)

        cmd.extend([
            "--build-arg", f"STEAM_USER={steam_user}",
            "--build-arg", f"STEAM_PASSWORD={steam_pass}"
        ])

    # Important: The context must be repo_root so Docker can see ros2_vive_controller/ and .ci/
    cmd.append(repo_root)

    try:
        subprocess.check_call(cmd)
        print(f"\n✅ Build successful: {full_image_name}")
    except subprocess.CalledProcessError:
        print("\n❌ Build failed.")
        sys.exit(1)

    # Optional Push
    if args.push:
        print(f"\n[PUSH] Pushing {full_image_name} to registry...")
        try:
            subprocess.check_call(["docker", "push", full_image_name])
            print(f"✅ Push successful.")
        except subprocess.CalledProcessError:
            print(f"\n❌ Push failed. Are you logged in to {args.registry}?")
            sys.exit(1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Build Multi-Stage Docker images for Vive Controller")

    parser.add_argument("--registry", default=DEFAULT_REGISTRY, help="Docker registry URL")

    # Matches your Dockerfile: base -> dep -> dev -> app
    parser.add_argument("--target", default="dev", choices=["base", "dep", "dev", "app"],
                        help="Build target stage (base, dep, dev, app)")

    parser.add_argument("--tag", help="Docker tag (default: reads from VERSION file)")
    parser.add_argument("--push", action="store_true", help="Push image to registry after build")

    # This was already in your args, but now it is used in the logic above
    parser.add_argument("--no-cache", action="store_true", help="Force rebuild of all layers (ignoring cache)")

    args = parser.parse_args()
    build_docker(args)