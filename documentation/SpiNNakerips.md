# Change SpiNNaker IP address

The SpiNNaker-5 board has two ethernet ports, one for management (located more central) and one for loading a model onto the cores (located more external). The management port can be used to change the IP address of both ports.

* Establish a connection to the management port. Either connect to the same local network, or make an ad-hoc connection with a direct cable connection and setting your own laptop IP to the same subnet (changing only the final value of the IP address). Make sure there is a connection using the ``ping`` command. The IP address of the management port could be:
  * 192.168.240.0 (default)
  * 10.0.0.70 (EDPR board)
* The spinnaker_tools package provides an application ``bmpc`` to communicate with the management port:

> bmpc 10.0.0.70

* A connection should be made, and you should be greated by the bmpc command prompt. Type ``help`` for information. Visualise the IP address of the board with:

> spin_ip

> bmp_ip

* Change the IP (and more with)

> spin_ip <Flag.X> <MAC.M> <ip_addr.P> <gw_addr.P> <net_mask.P> <port.D>

   for example

> spin_ip c059 00:00:a4:f0:00:01 10.0.0.7 10.0.0.1 255.255.0.0 17893

   and 

> bmp_ip c000 00:00:a4:f0:00:00 10.0.0.6 10.0.0.1 255.255.0.0 17893

* Restart the board for the new IP addresses to take affect.

