# Getting Started with `event-driven`

You may have some hardware in front of you that you are unsure of how to use, if not, and you just need help with software, skip ahead. If you have any questions with the these instructions, please open a github issue on the `event-driven` repository page.

## Hardware

Hardware could include sensors: ATIS, event-driven skin, cochlea, or IMU, as well as acquisition boards based on zynq chips: ZCB or z-turn. We'll assume the hardware is correctly connected and powered. However, you may need to

[set-up an sd-card for a ZCB or z-turn.](howtosetupSD.md)

If the system should be already ready-to-go, and you just need the board to start acquisition, you can follow these instructions to

[connect to the ZCB or z-turn.](connect_to_zcb.md)

Once you have a connection to the board, you need to

[set-up the yarpserver](setup_yarpserver.md)

to correctly communicate between your laptop and the board. Finally you will need to

[run `zynqGrabber`](zynqGrabber.md)

At this point, if you have checked your hardware is streaming events, you can start to read and process them, once your software is correctly installed on the laptop as well.

## Software

The first step is to install the dependencies  - typically just YARP, is required but we reccommend YCM, as well as openCV for visualisation - and the `event-driven` library itself. The

[instructions for installation](full_installation.md)

should be all you need to do to have a working environment. We suggest following the

[getting started with visualising events](1viewer.md)

tutorial to check everything is working, and to learn about the `yarpdataplayer`. If you can't get the tutorial working, double check everything is installed correctly, and if you can't find the problem, please open an issue on the `event-driven` repository page.

Finally, once you are ready, you can

[learn to write a basic processing module](example_module.md)

which covers, reading and writing events, and understanding event timestamps. Your module probably needs a nice way of

[visualising your output]()

and possibly needs to

[correct for lens disortion or external other external calibration]()

given you have

[calibrated your camera.](2calibration.md)

You can also learn to

[save and playback]()

data for offline processing and experiment re-creation.

