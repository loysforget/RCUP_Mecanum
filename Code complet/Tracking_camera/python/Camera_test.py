#!/usr/bin/python
# -*- coding: utf-8 -*-
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#####################################################
##           librealsense T265 example             ##
#####################################################

# First import the library
import pyrealsense2 as rs

#Initialisation de la caméra
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)
pipe.start(cfg)

X = []
Y = []
Z = []


try:
    for _ in range(1500):
        # Wait for the next set of frames from the camera
        frames = pipe.wait_for_frames()

        # Fetch pose frame
        pose = frames.get_pose_frame()
        if pose:
            # Print some of the pose data to the terminal²
            data = pose.get_pose_data()

            
            X = data.translation.x
            Y = -data.translation.z
            Z = data.translation.y
            print("Frame #{}".format(pose.frame_number))
            print("Position : x= {}, y= {}, z= {}".format(X,Y,Z))

finally:
    pipe.stop()

