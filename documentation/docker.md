## Setup docker environment
### Check udev rules in your host environment
```
ls /etc/udev/rules.d　-alh

# If you don't have `88-cyusb.rule` and `99-evkv2.rules` in this folder,
# Add them with following commands
sudo wget -P /etc/udev/rules.d https://raw.githubusercontent.com/prophesee-ai/openeb/main/hal_psee_plugins/resources/rules/88-cyusb.rules
sudo wget -P /etc/udev/rules.d https://raw.githubusercontent.com/prophesee-ai/openeb/main/hal_psee_plugins/resources/rules/99-evkv2.rules

# Reload udev rules in your host environment
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Build docker image
```
docker build --build-arg UID=$(id -u) --build-arg GID=$(id -g) -t event-driven:latest .
```
- Input current User ID and Group ID into Docker environment

### Run and enter docker container
```
docker run -it --privileged --network host -v /tmp/.X11-unix/:/tmp/.X11-unix -v /dev/bus/usb:/dev/bus/usb -e DISPLAY=unix$DISPLAY  --name event-driven event-driven:latest
```

### Open X Server for docker environment
```
xhost local:docker
```
