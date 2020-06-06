# -*- coding: utf-8 -*-

"""https://github.com/event-driven-robotics/importRosbag
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
importRpgDvsRos uses the importRosbag submodule
(original located at https://github.com/event-driven-robotics/importRosbag)
to import a rosbag, containing a selected set of ros msg types.
In particular, this supports the dvs_msgs/EventArray messages defined at:
http://rpg.ifi.uzh.ch/davis_data.html
It also supports some standard ros types:
    geometry_msgs/PoseStamped 
    geometry_msgs/Transform
    geometry_msgs/TransformStamped
    geometry_msgs/TwistStamped
    sensor_msgs/CameraInfo, (calibration)
    sensor_msgs/Image
    sensor_msgs/Imu 
    sensor_msgs/PointCloud2
    tf/tfMessage
Furthermore there is support for:
    esim_msgs/OpticFlow

It returns nested dicts of the form:
{
    info
    data
        channelName
            dvs
                "pol": numpy array of bool
                "x": numpy array of uint16
                "y": numpy array of uint16
                "ts": numpy array of float - seconds (basic format is int with unit increments of 80 ns) 
            frame ...
            imu ...
            etc ...

It optionally reorders the resulting data to support legacy templates (kwarg=template) 
Here follow 2 example templates:
    
template = {
    'left': {
        'dvs': '/davis/left/events',
    }, 'right': {
        'dvs': '/davis/right/events',
        }
    }    
    
template = {
    'ch0': {
        'dvs': '/dvs/events',
        'frame': '/dvs/image_raw',
        'pose6q': '/optitrack/davis',
        'cam': '/dvs/camera_info',
        'imu': '/dvs/imu'
        }
    }    

If a template is supplied, any connections which are not named in the template are not imported but simply listed.
If an empty template is supplied, then the contained types are printed out.
Use this to inspect a new .bag file before defining the import template.
Without any template, the default behaviour is to put any imported data into 
its own channel, named after the topic in the bag.
"""

#%%

import string

# Local imports
if __package__ is None or __package__ == '':
    from timestamps import zeroTimestampsForAChannel, rezeroTimestampsForImportedDicts
    from importRosbagSubmodule.importRosbag import importRosbag
else:
    from .timestamps import zeroTimestampsForAChannel, rezeroTimestampsForImportedDicts
    from .importRosbagSubmodule.importRosbag import importRosbag
    
def bimveeTypeForRosbagType(rosbagType):
    rosbagType = rosbagType.replace('/','_')
    if rosbagType == 'dvs_msgs_EventArray': return 'dvs'
    if rosbagType == 'esim_msgs_OpticFlow': return 'flowMap'
    if rosbagType == 'geometry_msgs_PoseStamped': return 'pose6q'
    if rosbagType == 'geometry_msgs_Transform': return 'pose6q'
    if rosbagType == 'geometry_msgs_TransformStamped': return 'pose6q'
    if rosbagType == 'geometry_msgs_TwistStamped': return 'twist'
    if rosbagType == 'sensor_msgs_CameraInfo': return 'cam'
    if rosbagType == 'sensor_msgs_Image': return 'frame'
    if rosbagType == 'sensor_msgs_Imu': return 'imu'
    if rosbagType == 'sensor_msgs_PointCloud2': return 'point3'
    if rosbagType == 'tf_tfMessage': return 'pose6q'
    return None

def importRpgDvsRos(filePathOrName, **kwargs):    
    template = kwargs.get('template')
    if template == {}: # Just list contents of bag without importing
        topics = importRosbag(filePathOrName=filePathOrName, listTopics=True, **kwargs)
        return
    topics = importRosbag(filePathOrName=filePathOrName, **kwargs)
    outDict = {
        'info': kwargs,
        'data': {}
            }
    outDict['info']['filePathOrName'] = filePathOrName
    if template is None:
        for topicLabel in topics.keys():
            rosbagType = topics[topicLabel].pop('rosbagType')
            bimveeType = bimveeTypeForRosbagType(rosbagType)
            if bimveeType is None:
                print('Actually, ' + topicLabel + ' has not been imported, because the rosbag message type ' + rosbagType + ' has not been recognised.')
            else:
                outDict['data'][topicLabel] = {bimveeType: topics[topicLabel]}
    else:
        # If we get to here then there is a template to parse
        # The template becomes the data branch of the importedDict
        for channelKey in template.keys():
            channelKeyStripped = str(channelKey).translate(str.maketrans('', '', string.punctuation))
            outDict['data'][channelKeyStripped] = {}
            for dataType in template[channelKey]:
                topicLabel = template[channelKey][dataType]
                topic = topics.pop(topicLabel)
                rosbagType = topic.pop('rosbagType')
                bimveeType = bimveeTypeForRosbagType(rosbagType)
                if bimveeType != dataType:
                    print('dataType "', dataType, '" not correctly defined for topic: "', topicLabel, '"')
                else:
                    outDict['data'][channelKeyStripped][dataType] = topic
    # Post processing
    if kwargs.get('zeroTime', kwargs.get('zeroTimestamps', True)):
        # Optional: start the timestamps at zero for the first event
        # This is done collectively for all the concurrent imports
        for channelKey in outDict['data'].keys():
            zeroTimestampsForAChannel(outDict['data'][channelKey])
        # jointly rezero for all channels
        rezeroTimestampsForImportedDicts(outDict)
    return outDict
