# --- Configuration ---
# Load variables from .env if it exists
ifneq ("$(wildcard .env)","")
	include .env
	export $(shell sed 's/=.*//' .env)
endif

# Default shell
SHELL := /bin/bash

# --- Phony Targets ---
.PHONY: help gui-perms build build-force push franka tiago tiago_pro g1 calibrate identify stop clean

# --- Help ---
help: ## Show this help message
	@echo "Vive Controller Docker Management"
	@echo "Usage: make <target>"
	@echo ""
	@echo "Available commands:"
	@awk 'BEGIN {FS = ":.*?## "} /^[a-zA-Z_-]+:.*?## / {printf "  \033[36m%-15s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)

# --- GUI Permissions ---
gui-perms: ## Set X11 permissions for Docker UI (used internally)
	@echo "Setting X11 permissions for Docker..."
	@xhost + > /dev/null

# --- Build Targets ---
build: ## Build the 'app' image (cached)
	docker compose build

build-force: ## Build from scratch (no cache)
	docker compose build --no-cache

push: ## Push built images to the registry
	@echo "Pushing images to the configured registry..."
	docker compose push

pull: ## Pull the latest pre-built images from the registry
	@echo "Pulling latest images..."
	docker compose pull

# --- Robot Missions ---
franka: gui-perms ## Start Franka (Single Right Controller)
	docker compose --profile franka up

tiago: gui-perms ## Start Tiago Dual Arm
	docker compose --profile tiago up

tiago_pro: gui-perms ## Start Tiago Pro Dual Arm
	docker compose --profile tiago_pro up

g1: gui-perms ## Start G1 Dual Arm
	docker compose --profile g1 up

# --- Utilities ---
calibrate: gui-perms ## Run workspace calibration
	# Runs calibration inside the franka container context
	docker compose --profile franka run --rm franka ros2 launch ros2_vive_controller calibration.launch.py

identify: ## Vibrate controllers to check IDs
	@echo "Vibrating RIGHT controller..."
	-docker exec -it ros2_vive_franka /entrypoint.sh ros2 service call /vive/right/identify std_srvs/srv/Trigger || \
	 docker exec -it ros2_vive_tiago /entrypoint.sh ros2 service call /vive/right/identify std_srvs/srv/Trigger || \
	 docker exec -it ros2_vive_tiago_pro /entrypoint.sh ros2 service call /vive/right/identify std_srvs/srv/Trigger || \
	 docker exec -it ros2_vive_g1 /entrypoint.sh ros2 service call /vive/right/identify std_srvs/srv/Trigger
	@echo "Vibrating LEFT controller..."
	-docker exec -it ros2_vive_tiago_pro /entrypoint.sh ros2 service call /vive/left/identify std_srvs/srv/Trigger || \
	 docker exec -it ros2_vive_g1 /entrypoint.sh ros2 service call /vive/left/identify std_srvs/srv/Trigger

stop: ## Stop all running vive containers
	docker compose down

clean: ## Remove all vive containers and networks
	docker compose down --remove-orphans --volumes

