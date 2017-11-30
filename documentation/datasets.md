# Datasets

Event-driven datasets contain the stream of events captured from a single or stereo event cameras. Each dataset can be played using the _yarpdataplayer_. Once the dataset is loaded in the _yarpdataplayer_ the specified output port (default=/zynqGrabber/vBottle:o) is opened and can be connected to any event-driven processing module, the event-driven preprocessing module, or directly to the _vFramer_.

Each datasets may also contain, other sensors on the iCub (also re-playable with the _yarpdataplayer_) and ground truth measurements, given the task (e.g. ball locations, corner locations). Also included are Matlab scripts to generate the result figures as in the relevant papers.

Datasets recorded with the DVS and ATIS have the following properties which must be set correctly, in the module parameters and the cmake options, to interpret the data:
> [DVS]
> width=128
> height=128
> VLIB_10BITCODEC=OFF
> VLIB_32BITTIME=OFF

> [ATIS]
> width=304
> height=240
> VLIB_10BITCODEC=ON
> VLIB_32BITTIME=OFF

## Ball Detection and Tracking

* 2 Datasets = hand-move, eye-move
* DVS
* ground truth supplied

[download](https://www.dropbox.com/s/8b0pp4o1qwufgl1/EDPR_DVS_BALLTRACKING.zip?dl=0)

## Corner Detection 

[insert link here]

