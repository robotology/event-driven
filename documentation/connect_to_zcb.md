# Connect to a ZCB or z-turn

You have a ZCB that is powered on and sitting in front of you and you want to read events from the camera/skin/cochlea that is attached to it. We want to make an `ssh` connection to it, connect to the `yarpserver`, and run `zynqGrabber`. Here are the steps:

First you are going to make a serial connection so you can configure the network connection. You need the network for `YARP` to function.

## Serial connection over (ubuntu instructions)
* Plug in a micro-usb cable from your laptop to the ZCB. Wait! There are two usb ports on the ZCB. Use the one that says uart next to it - it should be on the opposite side to the sd-card port.
```bash
sudo apt install screen
sudo screen /dev/ttyUSB0 115200
```
The login and password should be given to you by EDPR-IIT.

Okay now you have a terminal *inside the ZCB!* You need to decide how you are going to connect the ZCB to the network:

To exit `screen` do `ctrl+a` to open the list of commands,, then press `k`, then `y` to close the session.

### Option 1: External network

Connect both your own laptop and the ZCB to the same local network using an ethernet cable. The router should allocate the IP addresses and, if the network has internet connection, the ZCB should have connection too.

:warning: The IP address might change from time-to-time. Set-up the ZCB with a DHCP connection.

### Option 2: External network with Static IP

Connect both your own laptop and the ZCB to the same local network using an ethernet cable. Ask your network administrator for an available address to assign to the ZCB. It will have internet connectivety and the IP won't change. Set-up the ZCB with a static ip.

### Option 3: Ad-hoc with Static IP

Connect your laptop ethernet directly to the ZCB ethernet port.  The ZCB won't have internet connection, so if you need to configure the install/update the software, this option isn't valid - but it could be the easiest for a demo once you know the ZCB is already working. You'll need to set your own laptop as well as the ZCB to have a static IP.

### Option 4: Ad-hoc with Internet Connection Sharing

Connect your laptop ethernet directly to the ZCB ethernet port. With Internet connection sharing you should be able to share your laptops wifi connection to the ZCB over the ethernet. Your laptop assigns the IP so it can change every time you connect. To do so use run `nm-connection-editor` from terminal, press the `+` button to add a new connection, configure it to be ethernet and under `IPv4 Settings` use the "Shared to other computers" method. Name your connection something informative e.g. "ZCB connection". Set the ZCB to a dynamic IP address

## Setting the IP address of the ZCB

On the `screen` connection to the ZCB do the following:
```bash
nano /etc/network/interfaces
```
Add the following lines depending on the IP type:
- **Dynamic IP**
```bash
auto eth0
iface eth0 inet dhcp
hwaddress ether 00:0a:35:00:01:X
```
- **Static IP**
```bash
auto eth0
iface eth0 inet static
hwaddress ether 00:0a:35:00:01:X
address <ip address>
netmask 255.255.255.0
```
Save and exit `nano`.

```bash
sudo ifdown eth0
sudo ifup eth0
ifconfig
```

The `<ip address>` of the board should be reported. Take note so you can connect to the board over SSH.

## SSH connection

On your laptop
```bash
ping <ip address>
```
if a connection is found
```
ssh icub@<ip address>
```
The password should be given to you by EDPR-IIT.
