# soundDetectionAndLocalization application module

## Summary

This app implements a combination of two different tasks related to the auditory information. The iCub robot is able to recognize a set of sounds (pure tones). In addition, the robot is also able to localize where is that sound being produced. With those two features, a simple attention mechanism can be implemented in such a way the robot only will orientate its head when a predefined sound is being detected.

## Auditory model on iCub

The auditory model integrated within the iCub robot is called Neuromorphic Auditory Complex (NAC). It is based on two models: the Neuromorphic Auditory Sensor (NAS) and the Neuromorphic Superior Olivary Complex (NSOC). The first implements a model of the human cochlea, and the second implements a model of the human Superior Olivary Complex (SOC). The NAC model is a digital, event-based model designed to be run into FPGAs.

### Neuromorphic Auditory Sensor (NAS)

The NAS model integrated in the iCub robot is a stereo sensor with 32 frequency channels distributed in the range from 20 Hz to 22 kHz. Its architecture follows the biological distribution of the tonotopical frequency map of the human cochlea. This is, it can be implemented as a bank of bandpass filters connected in cascade. It performs a Fast Fourier Transform (FFT) operation over the input audio. The main feature of this model is that it carries out this operation in the spike-domain by using simple operations, thus reducing the design complexity. More information about the NAS can be found in its [official repository](https://github.com/RTC-research-group/OpenNAS).

### Neuromorphic Superior Olivary Complex (NSOC)

Talk about MSO and LSO.
The SOC model is based on the human SOC, that is composed by multiple nuclei. The main nuclei are the Media Superior Olive (MSO), the Lateral Superior Olive (LSO), the Anteroventral Cochlear Nucleus (AVCN) and the Medial Nucleus of the Trapezoid Body (MNTB). In the NSOC model, the MSO, LSO, and AVCN nuclei have been implemented. The AVCN performs the phase-lock operation, the MSO performs the Inter-aural Time Difference (ITD) extraction, and the LSO performs the Inter-aural Level Difference (ILD) extraction. As in the NAS implementation, the design strategy wat to use event-based processing blocks. The MSO is feed with the NAS' output events from frequency channels 13 to 16 (i.e. the frequency range is 592 Hz - 1166 Hz). And for each frequency channel there is a population of 16 neurons that implements the coincidence detector neurons defined in the Jeffress model of the MSO nucleus. The time detection range was set to 700 microseconds with an overlapping of 10 microseconds between neighbouring neurons. The LSO model is still under test. More information about the NSOC can be found in its [official repository](https://github.com/dgutierrezATC/nssoc).

## Modules

Next, a brief description of the modules which compose the application are showed.

### vCochleaEventsMapper

As it was aforementioned, the audio sensor integrated within the iCub has two different parts: the cochlea model and the sound source localization model. The events' addresses coming from the cochlea model are in the range [0, 127]. However, the events' addresses coming from the sound source localization model are not in the range [128, xxxx]. Therefore, the cochlea events mapper module remap the sound source localization events in order to have a continuous address range since it is needed for sending the events to SpiNNaker. This remap is fixed since it depends on the sensor architecture, which is fixed in the FPGA of the robot.
On the other hand, the spinnaker events mapper takes as input the SpiNNaker output events. Since the architecture of the output network running on SpiNNaker can change, we need to remap and split the SpiNNaker's output into two different networks: the sound classification and the sound source localization.
In addition, it could happen that the SpiNNaker machine is not available. In that case, the module cochlea events mapper has a flag to select if the SpiNNaker board is available, and it is used in the mapping task.

### vSpinnakerEventsMapper

The input information of this module is the output events comming from SpiNNaker (integer value representing the neuron ID, similar to AER). Since there are two different networks implemented on SpiNNaker, this module splits the output events from SpiNNaker into two different output ports. This way, it is easier to post-process the networks' responses in two different application modules. It's important to mention that if the SpiNNaker machine is not available, this module will not work and therefore the sound classification module will not work neither. However, it would be possible to connect the cochlea events mapper module directly to the auditory attention module and then perform the sound source localization. 

### vSoundClassification

This module takes the output events from the SpiNNaker events mapper module (if enabled) and generate a histogram with the events activity of the SpiNNaker's population that classify the input sound. After generating the histogram in a time bin, it calculate the maximum value of the histogram and set the associated sound as the winner. This histogram is showed by means of a viwer, where the winer neuron is indicated. For more information about this network, visit its [official repository](https://github.com/jpdominguez/Multilayer-SNN-for-audio-samples-classification-using-SpiNNaker).

### vAuditoryAttention

Similar to the sound classification module, this module takes the output events from the Inferior Colliculus (IC) network (if SpiNNaker is available) or from the cochlea events mapper module (if SpiNNaker is not available) and take a decission about the sound source localization based on a histogram. This module also plots a figure in which the reference is showed at the middle of the viewer, and a set of sound positions (based on the number of neurons set either on the SpiNNaker module or the events mapper module) are also plotted. Only the wining neuron will change its color to indicate the final decission.

### vRobotMovement

The aim of this module is to carry out a demo with either the simulated iCub (using Gazebo) or the real iCub. This module takes as input the winer neuron from the auditory attention module, which indicates the sound source position. Then, the robot will move both the head to that orientation, so the sound source will be in front of the robot's head. The movement's velocity can be configured, as well as if the user wants to use the real or the simulated robot. One thing to take into account is the hardware limitation of the head. The yaw range of the neck joint has a movement's range from -45 to 45 degrees. Therefore, the robot is not able to reach sound sources placed at, for example, 80 degrees. For doing that, the thorso has to be moved when the neck reaches its limit (still under development).
