# atis-bridge

A bridge application between Prophesee's x320 camera using the KTH micro interface and YARP

### Docker environment
1. Build dockerfile to generate docker image. It takes long time.
```
docker build  --build-arg UID=$(id -u) --build-arg GID=$(id -g) -f Dockerfile_Ubuntu2204_genx320 -t event-driven:ubuntu22.04-genx320  .
```


### Usage
1. Connect GenX320 camera via USB cable.
2. Confirm that this USB connection is detected by using this command
```
lsusb
```
You will notice that this USB device is mounted successfully
```
Bus XXX Device YYY: ID cafe:4001 TinyUSB TinyUSB Device
```
and, you can find mounted path as follows
```
ls /dev/ttyACM0
```

3. Deploy docker container with /dev/ttyACM0 mounted
```
docker run --rm -it --privileged --network host -v /tmp/.X11-unix/:/tmp/.X11-unix -v /dev/bus/usb:/dev/bus/usb -e DISPLAY=unix$DISPLAY -v /dev/ttyACM0:/dev/ttyACM0  --name event-driven event-driven:ubuntu22.04-genx320
```
By running this command, you can enter docker container.

4. Open another terminal, and enter docker container
```
docker exec -it event-driven /bin/bash
```

5. Run yarpserver in a terminal, and connect genx320 in the other terminal.
```
yarpserver &
```
```
x320-bridge
```

### Windonws: How to mount your GenX320 to WSL2
1. Open Powershell with administrator privilege
2. Install `usbipd-win`
```
winget install dorssel.usbipd-win
```
3. Repnen Powershell and check USB devices
```
usbipd list
```
You can find the USB serial device with the busid.
4. Share the USB device
```
usbipd bind --busid YOUR_BUSID
```
5. Attach the USB device to WSL
```
usbipd attach --wsl --busid YOUR_BUSID
```
6. WSL: you can find the mounted USB
```
lsusb
or
ls /dev/ttyACM*
```


### Specification
1. Resolution 320x320
