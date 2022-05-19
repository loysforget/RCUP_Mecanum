import numpy as np
import matplotlib.pyplot as plt
from rplidar import RPLidar
from math import pi

def get_data():
    lidar = RPLidar('COM4')
    for scan in lidar.iter_scans(max_buf_meas=1500):
        break
    lidar.stop()
    return scan

def getDistance(x1, x2, y1, y2):
    return np.sqrt(pow((x1 - x2),2) + pow((y1 - y2),2))

