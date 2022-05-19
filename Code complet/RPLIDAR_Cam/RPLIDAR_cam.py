import RPLIDAR_fcts as rp
import CamPoseData_fcts as cm
import numpy as np
from math import pi
import matplotlib.pyplot as plt

pipe = cm.initcam() 


current_data=rp.get_data()
for point in current_data:
        x.append(point[2]*np.sin(point[1]*pi/180)) #en mm
        y.append(point[2]*np.cos(point[1]*pi/180)) #en mm

 
try:
    while(True):
        # Wait for the next set of frames from the camera
        frames = pipe.wait_for_frames()

        # Fetch pose frame
        pose = frames.get_pose_frame()
        if pose:
            # Print some of the pose data to the terminal²
            data = pose.get_pose_data()
            print("Frame #{}".format(pose.frame_number))
            print("Position: {}".format(data.translation))
            x = data.translation.x*1000
            y = data.translation.y*1000
            z = data.translation.z*1000
            print("Velocity: {}".format(data.velocity))
            print("Acceleration: {}\n".format(data.acceleration))

finally:
    pipe.stop()
