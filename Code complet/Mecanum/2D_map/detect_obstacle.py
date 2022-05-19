import RPLIDAR_fcts as rp
import CamPoseData_fcts as cm
import math
import matplotlib.pyplot as plt
import numpy as np
from math import pi
import serial
import time



def write_read(x, arduino):
    """
    Communiquer avec le port série de la carte Arduino 
    input:
        x: commande à envoyer sous forme de chaine de caractère
        arduino: port serie
    output:
        data: informations recues de la parte de l'Arduino
    """

    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.05)
    data = arduino.readline()
    print(data)
    time.sleep(0.05)
    data = arduino.readline()
    print(data)
    time.sleep(0.05)
    data = arduino.readline()
    print(data)
    return data

def update_map(pipe):

    tmp = []
    x, y = [], []

    print(x)
    print(y)

    obx =[]
    oby =[]

    for i in range(4):
            print(i)
            current_data= rp.get_data()
            for point in current_data:
                if point[0]==15:
                   if(point[1]>=250 or point[1]<=110):
                        tmp = cm.get_pose(pipe)
                        x, y = tmp[0], tmp[1]
                        obx.append(x + (point[2]*np.sin(point[1]*pi/180))*0.001)
                        oby.append(y + (point[2]*np.cos(point[1]*pi/180))*0.001)
    return [obx,oby]

def ObstacleProche():
    for i in range(3):
            print(i)
            current_data= rp.get_data()
            for point in current_data:
                if point[0]==15:
                   if(point[1]>=250 or point[1]<=110):
                        if ((point[2]*0.001)<=0.8):
                            return True
    return False


show_animation = True
move_robot = True
ArduinoSerialPort = 'COM12' #Faut spécifier le port auquel est connecter l'Arduino ici


def main():
    print(__file__ + " start !!!")

    pipe = cm.initcam()

    if move_robot:
        arduino = serial.Serial(port=ArduinoSerialPort, baudrate=115200, timeout=.1) 
        write_read("#B",arduino)
        write_read("V",arduino)
        write_read("#B",arduino)
        write_read("R",arduino)
        write_read("r",arduino)


    ox, oy = [], []

    tmp1 = []
    tmp1 = update_map(pipe)
    ox.extend(tmp1[0])
    oy.extend(tmp1[1]) 

    plt.plot(ox, oy, ".k")
    plt.grid(True)
    plt.axis("equal")
    plt.ion()
    plt.show()

    while(True):
            tmp1 = []
            tmp1 = update_map(pipe)
            ox.extend(tmp1[0])
            oy.extend(tmp1[1]) 

            if show_animation:  # pragma: no cover
                plt.plot(ox, oy, ".k")
                #plt.plot(sx, sy, "og")
                plt.grid(True)
                plt.axis("equal")
                plt.draw()
                plt.pause(0.01)
                

            if move_robot:
                if(ObstacleProche()):
                    print("obstacle")
                    write_read("4M0W", arduino)
                    break
                else:
                    write_read("200yAt", arduino)

                

    
    print("end")
    write_read("4M0W", arduino)

    pass

if __name__ == '__main__':
    main()
