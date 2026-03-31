# --- Configuration ---
# Load variables from .env if it exists
ifneq ("$(wildcard .env)","")
    include .env
    export $(shell sed 's/=.*//' .env)
endif

# Default shell
SHELL := /bin/bash

# --- Help ---
.PHONY: help
help:
	@echo "Vive Controller Docker Management"
	@echo "Usage: make <target>"
	@echo ""
	@echo "Robot Missions:"
	@echo "  make franka      - Start Franka (Single Right Controller)"
	@echo "  make tiago       - Start Tiago Dual Arm"
	@echo "  make g1          - Start G1 Dual Arm"
	@echo ""
	@echo "Utility:"
	@echo "  make build       - Build the 'app' image (cached)"
	@echo "  make build-force - Build from scratch (no cache)"
	@echo "  make calibrate   - Run workspace calibration"
	@echo "  make identify    - Vibrate controllers to check IDs"
	@echo "  make stop        - Stop all running vive containers"
	@echo "  make clean       - Remove all vive containers and networks"

# --- GUI Permissions ---
.PHONY: gui-perms
gui-perms:
	@echo "Setting X11 permissions for Docker..."
	@xhost + > /dev/null

# --- Build Targets ---
.PHONY: build
build:
	docker compose build

.PHONY: build-force
build-force:
	docker compose build --no-cache

# --- Robot Missions ---
.PHONY: franka
franka: gui-perms
	docker compose --profile franka up

.PHONY: tiago
tiago: gui-perms
	docker compose --profile tiago up


.PHONY: tiago_pro
tiago_pro: gui-perms
	docker compose --profile tiago_pro up

.PHONY: g1
g1: gui-perms
	docker compose --profile g1 up

# --- Utilities ---
.PHONY: calibrate
calibrate: gui-perms
	# Runs calibration inside the franka container context
	docker compose --profile franka run --rm franka ros2 launch ros2_vive_controller calibration.launch.py

.PHONY: identify
identify:
	@echo "Vibrating RIGHT controller..."
	-docker exec -it ros2_vive_franka /entrypoint.sh ros2 service call /vive/right/identify std_srvs/srv/Trigger || \
	 docker exec -it ros2_vive_tiago /entrypoint.sh ros2 service call /vive/right/identify std_srvs/srv/Trigger || \
	 docker exec -it ros2_vive_tiago_pro /entrypoint.sh ros2 service call /vive/right/identify std_srvs/srv/Trigger || \
	 docker exec -it ros2_vive_g1 /entrypoint.sh ros2 service call /vive/right/identify std_srvs/srv/Trigger
	@echo "Vibrating LEFT controller..."
	-docker exec -it ros2_vive_tiago_pro /entrypoint.sh ros2 service call /vive/left/identify std_srvs/srv/Trigger || \
	 docker exec -it ros2_vive_g1 /entrypoint.sh ros2 service call /vive/left/identify std_srvs/srv/Trigger
.PHONY: stop
stop:
	docker compose down

.PHONY: clean
clean:
	docker compose down --remove-orphans --volumes