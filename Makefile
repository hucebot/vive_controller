# --- Configuration ---
# Load variables from .env if it exists
ifneq ("$(wildcard .env)","")
	include .env
	export $(shell sed 's/=.*//' .env)
endif

# Default shell
SHELL := /bin/bash

# --- Phony Targets ---
.PHONY: help gui-perms build build-force push pull setup setup-demo teleop drivers franka tiago tiago_pro g1 calibrate identify stop clean

# Port of the setup web app (make setup)
SETUP_PORT ?= 8800
export SETUP_PORT

# --- Help ---
help: ## Show this help message
	@echo "Vive Controller Docker Management"
	@echo "Usage: make <target>"
	@echo ""
	@echo "Available commands:"
	@awk 'BEGIN {FS = ":.*?## "} /^[a-zA-Z0-9_-]+:.*?## / {printf "  \033[36m%-15s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)

# --- GUI Permissions ---
gui-perms: ## Set X11 permissions for Docker UI (used internally)
	@echo "Setting X11 permissions for Docker..."
	@xhost + > /dev/null

# --- Build Targets ---
# All controller robots share one image, so building/pushing/pulling it through one service is enough.
IMAGE_SERVICE := drivers

build: ## Build the 'app' image (cached)
	docker compose --profile $(IMAGE_SERVICE) build $(IMAGE_SERVICE)

build-force: ## Build from scratch (no cache)
	docker compose --profile $(IMAGE_SERVICE) build --no-cache $(IMAGE_SERVICE)

push: ## Push built images to the registry
	@echo "Pushing images to the configured registry..."
	docker compose --profile $(IMAGE_SERVICE) push $(IMAGE_SERVICE)

pull: ## Pull the latest pre-built images from the registry
	@echo "Pulling latest images..."
	docker compose --profile $(IMAGE_SERVICE) pull $(IMAGE_SERVICE)

# --- Setup ---
setup: ## Web app to pick the controllers, robot and origin, and paint the workspace (writes config/station.yaml)
	@scripts/vive_run.sh -it python3 -m ros2_vive_controller.setup_app

setup-demo: ## Try the setup app with simulated controllers (no hardware needed, saves nothing)
	docker compose --profile drivers run --rm --no-deps drivers python3 -m ros2_vive_controller.setup_app --demo

teleop: ## Start the robot chosen in make setup (robot: in config/station.yaml)
	@robot=$$(sed -n 's/^robot:[[:space:]]*//p' config/station.yaml 2>/dev/null | tr -d "'\" "); \
	if [ -z "$$robot" ]; then echo "No robot in config/station.yaml: run make setup first, or make <robot>."; exit 1; fi; \
	echo "Starting $$robot (robot in config/station.yaml)"; \
	$(MAKE) --no-print-directory $$robot

# --- Robot Missions ---
drivers: gui-perms ## Start only the Vive drivers + RViz (no robot)
	docker compose --profile drivers up

franka: gui-perms ## Start Franka (Single Right Controller)
	docker compose --profile franka up

tiago: gui-perms ## Start Tiago Dual Arm
	docker compose --profile tiago up

tiago_pro: gui-perms ## Start Tiago Pro Dual Arm
	docker compose --profile tiago_pro up

g1: gui-perms ## Start G1 Dual Arm
	docker compose --profile g1 up

# --- Utilities ---
calibrate: gui-perms ## Paint the right arm's workspace fence in RViz (saved in config/station.yaml); SIDE=left for the left arm
	# Same controllers and origin frame as teleop (config/station.yaml, then .env), so the fence is in the same frame
	docker compose --profile drivers run --rm drivers ros2 launch ros2_vive_controller calibration.launch.py side:=$(or $(SIDE),right)

identify: ## List the connected Vive devices and vibrate each controller (no robot needed)
	@scripts/vive_run.sh python3 -m ros2_vive_controller.identify

stop: ## Stop all running vive containers
	docker compose down

clean: ## Remove all vive containers and networks
	docker compose down --remove-orphans --volumes

