import numpy as np
import matplotlib.pyplot as plt
from rplidar import RPLidar
from math import pi
import serial
import time

arduino = serial.Serial(port='COM10', baudrate=115200, timeout=.1)
def write_read(x):
    arduino.write(bytes(x, 'utf-8'))
    pass


def getDistance(x1, x2, y1, y2):
    return np.sqrt(pow((x1 - x2),2) + pow((y1 - y2),2))

def get_data():
    lidar = RPLidar('COM4')
    for scan in lidar.iter_scans(max_buf_meas=3000):
        break
    lidar.stop()
    lidar.disconnect()
    return scan

angle_min = 300*pi/180 #5,23599
angle_max = 60*pi/180 #1.047
while(True):
    current_data=get_data()
    for point in current_data:
        if point[0]==15:
            angle = point[1]*pi/180
            x = point[2]*np.sin(point[1]*pi/180) #en mm
            y = point[2]*np.cos(point[1]*pi/180) #en mm
            print(angle)
            print(getDistance(x, 0, y, 0))
            if((angle >= angle_min  and angle <= angle_max ) and  getDistance(x, 0, y, 0) <= 300):
                write_read("Hi")
                print("Hi")
            else:
                write_read("Ho")
                print("Ho")
