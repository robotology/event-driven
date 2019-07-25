# Datasets

Event-driven datasets contain the stream of events captured from a single or stereo event cameras. Each dataset can be played using the _yarpdataplayer_. Once the dataset is loaded in the _yarpdataplayer_ the specified output port (default=/zynqGrabber/AE:o) is opened and can be connected to any event-driven processing module, the event-driven preprocessing module, or directly to the _vFramerLite_.

Each datasets may also contain, other sensors on the iCub (also re-playable with the _yarpdataplayer_) and ground truth measurements, given the task (e.g. ball locations, corner locations). Also included are Matlab scripts to generate the result figures as in the relevant papers.

Datasets recorded with the DVS and ATIS have the following properties which must be set correctly, in the module parameters and the cmake options, to interpret the data:
> [DVS]
> width=128
> height=128
> VLIB_CODEC_TYPE=CODEC_128x128
> VLIB_TIMER_BITS=24
> VLIB_CLOCK_PERIOD_NS=128

> [ATIS_20]
> width=304
> height=240
> VLIB_CODEC_TYPE=CODEC_304x240_20
> VLIB_TIMER_BITS=24
> VLIB_CLOCK_PERIOD_NS=80

> [ATIS_24]
> width=304
> height=240
> VLIB_CODEC_TYPE=CODEC_304x240_24
> VLIB_TIMER_BITS=30
> VLIB_CLOCK_PERIOD_NS=80

## Sample Dataset

* simple dataset to visualise
* ATIS_24

[download](https://doi.org/10.5281/zenodo.2556755)

## Ball Detection and Tracking

* 2 Datasets = hand-move, eye-move
* DVS
* ground truth supplied

[download](https://figshare.com/s/0abd8f18312bec15b121)

## Corner Detection

* 2 Datasets = checkboard (multiple different speeds and angles), naturalscene
* DVS
* ground truth supplied for naturalscene

[download](https://figshare.com/s/0abd8f18312bec15b121)

## VVV18-EVENTDRIVEN-DATASET

* 3 Datasets = 1, 2, 3 with different motions of the object and the robot
* robot encoder positions for head and torso supplied, stereo ATIS output, stereo RGB camera supplied
* scripts to recreate the movement of the robot supplied
* ATIS_20

[download](https://figshare.com/s/0abd8f18312bec15b121)

## Parallel Visual Tracking

* 10 datasets with moving circular target
* contains ground truth and outputs of tracking algorithm run on the CPU and on the SpiNNAker
* ATIS_24

[download](https://doi.org/10.5281/zenodo.2556755)



