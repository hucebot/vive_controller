This file provides instructions for setting up and using the Vive Controller package without Docker.

## Prerequisites
The following dependencies are required to use this package without Docker:
- ROS1 or ROS2 (depending on your preference)
- Steam Installed and configured
- Python libraries requeried: Refer to the `./install/install_python_libs.sh` script
- Dependencies required: Refer to the `./install/install_system_libs.sh` script
- [OpenVR](https://github.com/ValveSoftware/openvr)
- **Hardware**:
  - Vive Tracker (optional)
  - Vive Joystick (optional)
  - Vive Base Station (mandatory, at least one)

## Installation

- Install the required system libraries by running the following script:
```bash
sh ./install/install_system_libs.sh
```

- Install the required Python libraries by running the following script:
```bash
sh ./install/install_python_libs.sh
```

- Install Steam and SteamVR.

## Usage
## Steam VR Instructions
### Install Steam
The easiest way to install Steam is to install through a command line. To do this, run the following command:
```bash
sudo apt install steam
```

Once Steam is installed, open it and log in with your account. If you don't have an account, you can create one for free. Then, go to the Steam Store and search for SteamVR. Install it. 
Then you need to RUN the BETA Version.

### Using without headset

Basically you need to edit the following files:
- **default.vrsettings** located in **Steam Directory**/steamapps/common/SteamVR/drivers/null/resources/settings/default.vrsettings
  - On the null driver secction replace: **"enable": false** to **"enable": true**
- **default.vrsettings** located in **Steam Directory**/steamapps/common/SteamVR/resources/settings/default.vrsettings
  - On the steamvr section replace: **"requireHmd": true** to **"requireHmd": false**
  - On the steamvr section replace: **"forcedDriver": ""** to **"forcedDriver": "null"**
  - On the steamvr section replace: **"activateMultipleDrivers": false** to **"activateMultipleDrivers": true**

We prepare command that already do this. Move to the folder `.install` and the run the following command:
```bash
sh replace.sh path-where-steam-is-installed #For example: sh replace.sh ~/.steam/debian-installation
```