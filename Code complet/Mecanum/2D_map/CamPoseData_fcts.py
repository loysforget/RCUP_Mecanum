#!/usr/bin/python
# -*- coding: utf-8 -*-
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#####################################################
##           librealsense T265 example             ##
#####################################################

# First import the library
import pyrealsense2 as rs

def initcam():
    # Declare RealSense pipeline, encapsulating the actual device and sensors
    pipe = rs.pipeline()
    # Build config object and request pose data
    cfg = rs.config()
    cfg.enable_stream(rs.stream.pose)

    # Start streaming with requested config
    pipe.start(cfg)
    
    return pipe
    
def get_pose(pipe):
    # Wait for the next set of frames from the camera
    frames = pipe.wait_for_frames()

    # Fetch pose frame
    pose = frames.get_pose_frame()

    if pose:
        # Print some of the pose data to the terminal
        data = pose.get_pose_data()
        x = round(data.translation.x, 4)
        y = -round(data.translation.z, 4)
        z = round(data.translation.y, 4) 
        #print("Position : x= {}, y= {}, z= {}".format(x,y,z))

    return [x, y]

