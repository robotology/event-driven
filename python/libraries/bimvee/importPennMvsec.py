# -*- coding: utf-8 -*-

"""
Copyright (C) 2019 Event-driven Perception for Robotics
Authors: Sim Bamford
This program is free software: you can redistribute it and/or modify it under 
the terms of the GNU General Public License as published by the Free Software 
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY 
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with 
this program. If not, see <https://www.gnu.org/licenses/>.

Intended as part of bimvee (Batch Import, Manipulation, Visualisation and Export of Events etc)
importPennMvsec imports from Penn's MVSEC data sets, compatible with importAe function.
https://daniilidis-group.github.io/mvsec/
A first approach opened the hdf5 file. That function is retained here commented below.
The current approach uses the rosbags instead; 
it is a wrapper for the importRpgDvsRos function which applies a default template

TODO: how to import the "equidistant" camera model properly?
Not yet handled:
    
/velodyne_point_cloud (sensor_msgs/PointCloud2) - Point clouds from the Velodyne (one per spin).
/visensor/.../image_raw (sensor_msgs/Image) - Grayscale images from the left VI-Sensor camera.
/visensor/imu (sensor_msgs/Imu) - IMU readings from the VI-Sensor.
/visensor/cust_imu (visensor_node/visensor_imu) - Full IMU readings from the VI-Sensor (including magnetometer, temperature and pressure).
/davis/.../depth_image_raw (sensor_msgs/Image) - Depth maps for the left DAVIS camera at a given timestamp (note, these images are saved using the CV_32FC1 format (i.e. floats).
/davis/.../depth_image_rect (sensor_msgs/Image) - Rectified depth maps for the left DAVIS camera at a given timestamp (note, these images are saved using the CV_32FC1 format (i.e. floats).
/davis/.../blended_image_rect (sensor_msgs/Image) - Visualization of all events from the left DAVIS that are 25ms from each left depth map superimposed on the depth map. This message gives a preview of what each sequence looks like.
/davis/.../odometry (geometry_msgs/PoseStamped) - Pose output using LOAM. These poses are locally consistent, but may experience drift over time. Used to stitch point clouds together to generate depth maps.
/davis/.../pose (geometry_msgs/PoseStamped) - Pose output using Google Cartographer. These poses are globally loop closed, and can be assumed to have minimal drift. Note that these poses were optimized using Cartographer’s 2D mapping, which does not optimize over the height (Z axis). Pitch and roll are still optimized, however.
"""

if __package__ is None or __package__ == '':
    from importRpgDvsRos import importRpgDvsRos
else:
    from .importRpgDvsRos import importRpgDvsRos
    
def importPennMvsecDavis(**kwargs):
    if kwargs.get('template') is None:
        kwargs['template'] = {
            'davisLeft': {
                'dvs': '/davis/left/events',
                'frame': '/davis/left/image_raw',
                'imu': '/davis/left/imu',
                'cam': '/davis/left/camera_info',
            }, 'davisRight': {
                'dvs': '/davis/right/events',
                'frame': '/davis/right/image_raw',
                'imu': '/davis/right/imu',
                'cam': '/davis/right/camera_info',
                }
            }
    return importRpgDvsRos(**kwargs)

# TODO: There are other elements of the ground truth for various files, not handled in this template
def importPennMvsecGt(**kwargs): # Ground truth
    if kwargs.get('template') is None:
        kwargs['template'] = {
            'poseLocal': {
                'pose6q': '/davis/left/odometry',
            }, 'poseGlobal': {
                'pose6q': '/davis/left/pose',
            }, 'depthLeft': {
                'frame': '/davis/left/depth_image_raw',
            }, 'depthRight': {
                'frame': '/davis/right/depth_image_raw',
                }
            }
    return importRpgDvsRos(**kwargs)

'''
import h5py
import numpy as np

def importFromHdf5(**kwargs):
    Opens a .hdf5 file in param pathOrFileName
    Format defined here:
    https://daniilidis-group.github.io/mvsec/download/    
    We expect the following format - the hdf5 file opens into a dict as:
    {   velodyne
        davis
            left
                events
                image_raw
                image_raw_event_inds
                image_raw_ts
                imu
                imu_ts
            right ...
    ... where each of the above fields follows the appropirate rosbag format,
    and in particular, events follows rpg_dvs_ros events format. 
    This format is documented at: http://rpg.ifi.uzh.ch/davis_data.html

    file = h5py.File(kwargs.get('filePathOrName'), 'r')
    channels = file['davis'].keys()
    # Expecting 'left' and 'right' here
    outputDict = {
        'info': kwargs,
        'data': {}}
    
    for channel in channels:
        print('Exploring channel: ' + str(channel))
        outputDict['data'][channel] = {}
        currentChannelDict = outputDict['data'][channel]
        try:
            eventData = file['davis'][channel]['events']
            currentChannelDict['pol'] = {
                'x': np.asarray(eventData[:, 0], dtype=np.int16), 
                'y': np.asarray(eventData[:, 1], dtype=np.int16), 
                'ts': np.asarray(eventData[:, 2] - np.min(eventData[:, 2]), dtype=np.float64), 
                'pol': np.asarray(eventData[:, 3] == 1, dtype=np.bool), 
                }    
        except KeyError:
            print('No event data found for channel ' + str(channel))
         #'image_raw',
         #'image_raw_event_inds',
         #'image_raw_ts',
         #'imu',
         #'imu_ts']

    return outputDict
'''
    



