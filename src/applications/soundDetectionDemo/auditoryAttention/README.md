# soundDetectionAndLocalization application module

## Summary

This app implements a combination of two different tasks related to the auditory information. The iCub robot is able to detect a set of sounds (pure tones). In addition, the robot is also able to localize the where is that sound being produced. With those two features, a simple attention mechanism can be implemented in such a way the robot only will orientate its head when a predefined sound is being detected.

## Auditory model on iCub

Brief explanation about the model implemented on iCub.

### Neuromorphic Auditory Sensor (NAS)

Talk about number of frequency channels, stereo, features...

### Superior Olivary Complex (SOC)

Talk about MSO and LSO.

## Modules

Next, a brief description of the modules which compose the application are showed.

### vCochleaEventsMapper

Why is this module important.

### SpiNNaker code

The pure tones classification and the localization are computed here.

### vAuditoryAttention

Take the info from SpiNNaker and take a decission.