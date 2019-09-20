# Setup the a `yarpserver`

The most important thing is to make sure all computers that are using `YARP` together are on the same subnet, and that the `yarpserver` is running on the same sub-net. Most problems with connection are because one of these is not true. Here's the standard way to do if for `event-driven`.

#### run the `yarpserver` on the ZCB

On you SSH connection to the ZCB:

> yarp namespace /\<your name\>

> yarp conf \<ip address\> 10000

> yarpserver

where \<ip address\> is the IP address assigned to the [ZCB eth0 connection.](connect_to_zcb.md) If you want to run the `yarpserver` and the `zynqGrabber` on the same SSH connection, you can run the `yarpserver` in the background:

> yarpserver &

you can use

> fg

to bring `yarpserver` back to the foreground.

#### connect your laptop to the `yarpserver`

On a terminal on your own laptop:

> yarp conf \<ip address\> 10000

*Note: this is the ip address of the ZCB! not your laptop!

> yarp detect

should find the `yarpserver` you have running on the zcb

