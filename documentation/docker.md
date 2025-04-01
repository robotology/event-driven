## Setup docker environment
### For Ubuntu - Check udev rules in your host environment
```
ls /etc/udev/rules.d　-alh

# If you don't have `88-cyusb.rule` and `99-evkv2.rules` in this folder,
# Add them with following commands
# Otherwise, you can skip these commands
sudo wget -P /etc/udev/rules.d https://raw.githubusercontent.com/prophesee-ai/openeb/main/hal_psee_plugins/resources/rules/88-cyusb.rules
sudo wget -P /etc/udev/rules.d https://raw.githubusercontent.com/prophesee-ai/openeb/main/hal_psee_plugins/resources/rules/99-evkv2.rules

# Reload udev rules in your host environment
sudo udevadm control --reload-rules
sudo udevadm trigger
```
### For Windows - install camera plugins
1. Download [wdi-simple.exe from our file server](https://kdrive.infomaniak.com/app/share/975517/4f59e852-af5e-4e00-90fc-f213aad20edd)
2. Execute the following commands in a Command Prompt launched as an administrator:

```bash
wdi-simple.exe -n "EVK" -m "Prophesee" -v 0x04b4 -p 0x00f4
wdi-simple.exe -n "EVK" -m "Prophesee" -v 0x04b4 -p 0x00f5
wdi-simple.exe -n "EVK" -m "Prophesee" -v 0x04b4 -p 0x00f3
```

### Check hardware configuration
Firmware
- To use later version of Metavision SDK than V3.1.2, the Firmware of EVK3 and EVK4 is required to be at least in version 3.9
  - Check this [Release Notes](https://docs.prophesee.ai/stable/release_notes.html#v3-1-2-change-logs-16-12-2022)

Ubuntu version
- To use Ubuntu 22.04, Metavision SDK should be at least in version 4.0
  - This means firmware should be at least in version 3.9
- The earlier version of Metavision SDK than 4.0 does not support Ubuntu 22.04.
  - Should use Ubuntu 20.04

### Build docker image
Environment: Ubuntu 20.04 + Metavision SDK 3.0 + EVK firmware (< 3.9)
```
docker build --build-arg UID=$(id -u) --build-arg GID=$(id -g) -f Dockerfile_Ubuntu2004 -t event-driven:ubuntu20.04 .
```
Environment: Ubuntu 22.04 + Metavision SDK 4.6 + EVK firmware (>= 3.9)
```
docker build --build-arg UID=$(id -u) --build-arg GID=$(id -g) -f Dockerfile_Ubuntu2204 -t event-driven:ubuntu22.04 .
```
- Input current User ID and Group ID into Docker environment

### Run and enter docker container
```
docker run -it --privileged --network host -v /tmp/.X11-unix/:/tmp/.X11-unix -v /dev/bus/usb:/dev/bus/usb -e DISPLAY=unix$DISPLAY  --name event-driven event-driven:ubuntu20.04
```
- To use Ubuntu 22.04, change the last part of command to `event-driven:ubuntu22.04`.

### Open X Server for docker environment
```
xhost local:docker
```
