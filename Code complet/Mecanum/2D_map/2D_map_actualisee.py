import RPLIDAR_fcts as rp
import CamPoseData_fcts as cm
import numpy as np
from math import pi
import matplotlib.pyplot as plt 
import serial
import time


def write_read(x, arduino):
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

def update_map(pipe, angle =0):

    tmp = []
    x, y = [], []

    print(x)
    print(y)

    obx =[]
    oby =[]

    for i in range(2):
            print(i)
            current_data= rp.get_data()
            for point in current_data:
                if point[0]==15:
                   if(point[1]>=240 or point[1]<=100):
                    tmp = cm.get_pose(pipe)
                    x, y = tmp[0], tmp[1]
                    obx.append(x + (point[2]*np.sin((point[1]+abs(angle))*pi/180))*0.001)
                    oby.append(y + (point[2]*np.cos((point[1]+abs(angle))*pi/180))*0.001)
    return [obx,oby]

show_animation = True
move_robot = True
virage1 = False
virage2 = False
ArduinoSerialPort = 'COM12'


def main():
    print("start !!!")

    pipe = cm.initcam()

    if move_robot:
        arduino = serial.Serial(port=ArduinoSerialPort, baudrate=115200, timeout=.1)
        write_read("#B",arduino)
        write_read("V",arduino)
        write_read("#B",arduino)
        write_read("R",arduino)
        write_read("r",arduino)

    cm.get_pose(pipe) #initial pose

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

    if move_robot:
        for i in range(11):
            i_tmp = 300*i
            i_str= str(i_tmp)

            write_read("0"+ "xAt/"+ i_str + "yAt", arduino)

            tmp1 = []
            tmp1 = update_map(pipe)
            ox.extend(tmp1[0])
            oy.extend(tmp1[1])
            #time.sleep(1)
            plt.plot(ox, oy, ".k")
            plt.grid(True)
            plt.axis("equal")
            plt.draw()
            plt.pause(0.05)

        if virage1:
            write_read("0" + "xAt/"+ "3000" + "yAt/" + "-93" +"zAt", arduino)
            time.sleep(3)
            tmp1 = []
            tmp1 = update_map(pipe,-90)
            ox.extend(tmp1[0])
            oy.extend(tmp1[1])
            plt.plot(ox, oy, ".k")
            plt.grid(True)
            plt.axis("equal")
            plt.draw()
            plt.pause(0.05)    
            for i in range(11):
                i_tmp = 300*i
                i_str= str(i_tmp)

                write_read(i_str+ "xAt/"+ "3000" + "yAt", arduino)

                tmp1 = []
                tmp1 = update_map(pipe,-90)
                ox.extend(tmp1[0])
                oy.extend(tmp1[1])
                plt.plot(ox, oy, ".k")
                plt.grid(True)
                plt.axis("equal")
                plt.draw()
                plt.pause(0.05)

        if virage2:
            write_read("3000" + "xAt/"+ "3000" + "yAt/" + "0" +"zAt", arduino)
            time.sleep(5)

            for i in range(7):
                i_tmp = 3000 + 300*i
                i_str= str(i_tmp)

                write_read("3000"+ "xAt/"+ i_str + "yAt", arduino)

                tmp1 = []
                tmp1 = update_map(pipe)
                ox.extend(tmp1[0])
                oy.extend(tmp1[1])
                plt.plot(ox, oy, ".k")
                plt.grid(True)
                plt.axis("equal")
                plt.draw()
                plt.pause(0.05)

        """
        Les prochaines lignes sert à revenir
        à la position de départ

        """
        if virage2:
            for i in range(7):
                i_tmp = 4800 - 300*i
                i_str= str(i_tmp)

                write_read("3000" + "xAt/"+ i_str + "yAt", arduino)
                time.sleep(1)

            write_read("3000"+ "xAt/"+ "3000" + "yAt/" + "-93" + "zAt", arduino)
            time.sleep(3)

        if virage1:
            for i in range(11):
                i_tmp = 3000 - 300*i
                i_str= str(i_tmp)

                write_read(i_str+ "xAt/"+ "3000" + "yAt", arduino)
                time.sleep(1)

            write_read("0"+ "xAt/"+ "3000" + "yAt/" + "0" + "zAt", arduino)
            time.sleep(3)

        for i in range(11):
            i_tmp = 3000 - 300*i
            i_str= str(i_tmp)

            write_read("0" + "xAt/"+ i_str + "yAt", arduino)
            time.sleep(1)

        if show_animation:  # pragma: no cover
            plt.plot(ox, oy, ".k")
            plt.grid(True)
            plt.axis("equal")
            plt.ioff()
            plt.show()
        
        

    print("End !!!")

    pass

if __name__ == '__main__':
    main()