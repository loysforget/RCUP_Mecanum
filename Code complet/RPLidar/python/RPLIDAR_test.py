import numpy as np
import matplotlib.pyplot as plt
from rplidar import RPLidar
import rplidar
from math import pi

def get_data():
    lidar = RPLidar('COM4') #Faut spécifier le port auquel est connecter le Lidar ici
    lidar.motor_speed = rplidar.MAX_MOTOR_PWM
    for scan in lidar.iter_scans(min_len=100, max_buf_meas=500):
        break
    lidar.stop()
    return scan

for i in range(2):
    if(i%7==0):
        x = []
        y = []
    print(i)
    current_data=get_data()
    for point in current_data:
        if point[0]==15:
            x.append(point[2]*np.sin(point[1]*pi/180))
            y.append(point[2]*np.cos(point[1]*pi/180))

    plt.plot(x, y, ".k")
    plt.grid(True)
    plt.axis("equal")
    plt.ion()
    plt.show()



    """"
    plt.clf()
    plt.scatter(x, y)
    plt.pause(.1)
    """
plt.ioff()
plt.show()
