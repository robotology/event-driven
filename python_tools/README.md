# Offline Python Scripts

`event-driven` has two companion repositories for offline dataset manipulation which has replaced much of the python helper functions:

### BIMVEE

[Batch Import, Manipulation, Visualisation, and Export of Events](https://github.com/event-driven-robotics/bimvee)

Loading and converting datasets from most of the common event datasets

### MUSTARD

[MUlti STream Agnostic Representation Dataplayer](https://github.com/event-driven-robotics/mustard)

Uses BIMVEE to load any dataset and visualise with tracking bar. Can be used to annotate data


### Scripts

Here we have a couple of scripts:

`ev2converter.py` - uses BIMVEE to convert old event-driven datasets to the new event-driven-2.0 format.

`plot_imu_dump.py` - datasets dumped from imu calibration methods can be visualised to understand the data quality.


