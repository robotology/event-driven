Look at your namespace to not cause conflicts with others: 
`yarp namespace`

And set your personal one:
`yarp namespace /arren`

Where the conf file is:
`yarp conf`

Look at the IP:
`cat $(yarp conf)`

Look at the IP of the ethernet cable:
`ip -c -h a`

Set the yarp IP yarpserver will connect to: 
`yarp conf IP_ETHERNET_CABLE 10000`

Type:
`yarpserver`

Open a new terminal and run the camera: 
`atis-bridge-sdk --help`

If you are using gen3, use ubuntu 20, so check your version: 
`lsb-release -a`

Check the camera port: 
`yarp name list`

Visualise the data: 
`vFramer --src /atis3/AE:o --iso --width 640 --height 480`

You can visualise different representations, so type the following for the available options: 
`vFramer --help`

To start recording data, open: 
`yarpdatadumper --help`

An example is, not creating latencies:
`yarpdatadumper --txTime --name /left --dir /dump/experiment/ATIS`

Twice if you want stereo:
`yarpdatadumper --txTime --name /right --dir /dump/experiment/ATIS2`

Connect to the port to stream events in the recording ports: 
`yarp connect /atis3/AE:o /left fast_tcp && yarp connect /atis3/AE:o /right fast_tcp`

Disconnect or CTRL-C in the dumper:
`yarp disconnect /atis3/AE:o /left fast_tcp && yarp connect /atis3/AE:o /right fast_tcp`

Playback data haing checked yarp was installed with Qt dpeendency: 
`yarpdataplayer`

Open the folder at the level it contains ATIS and ATIS2, not inside each folder. Once loaded, press play. 







