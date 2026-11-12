# --- Configuration ---
# Load variables from .env if it exists
ifneq ("$(wildcard .env)","")
	include .env
	export $(shell sed 's/=.*//' .env)
endif

# Default shell
SHELL := /bin/bash

# --- Phony Targets ---
.PHONY: help gui-perms build build-force push pull drivers franka tiago tiago_pro g1 calibrate identify stop clean

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
calibrate: gui-perms ## Paint the workspace fence with the right controller (writes config/workspace.yaml)
	# Same serial and reference lighthouse as teleop, so the fence is recorded in the same frame
	docker compose --profile drivers run --rm drivers ros2 launch ros2_vive_controller calibration.launch.py \
		serial:=$(strip $(SERIAL_RIGHT)) reference_lighthouse_serial:=$(strip $(REFERENCE_LIGHTHOUSE_SERIAL))

IDENTIFY_ENV := -e SERIAL_RIGHT="$(strip $(SERIAL_RIGHT))" -e SERIAL_LEFT="$(strip $(SERIAL_LEFT))" \
	-e REFERENCE_LIGHTHOUSE_SERIAL="$(strip $(REFERENCE_LIGHTHOUSE_SERIAL))"

identify: ## List the connected Vive devices and vibrate each controller (no robot needed)
	@c=$$(docker ps --format '{{.Names}}' | grep -m1 -E '^ros2_vive_(franka|tiago|tiago_pro|g1|drivers)$$'); \
	if [ -n "$$c" ]; then \
		docker exec $(IDENTIFY_ENV) $$c /entrypoint.sh python3 -m ros2_vive_controller.identify; \
	else \
		docker compose --profile drivers run --rm --no-deps $(IDENTIFY_ENV) drivers python3 -m ros2_vive_controller.identify; \
	fi

stop: ## Stop all running vive containers
	docker compose down

clean: ## Remove all vive containers and networks
	docker compose down --remove-orphans --volumes

