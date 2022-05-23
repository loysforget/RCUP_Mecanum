import RPLIDAR_fcts as rp
import CamPoseData_fcts as cm
import numpy as np
from math import pi
import matplotlib.pyplot as plt 
import serial
import time

def get_pose(pipe):
    # Wait for the next set of frames from the camera
    frames = pipe.wait_for_frames()

    # Fetch pose frame
    pose = frames.get_pose_frame()
    if pose:
        # Print some of the pose data to the terminal²
        data = pose.get_pose_data()
        #print("Position: {}".format(data.translation))
        x = round(data.translation.x, 3)
        y = -round(data.translation.z, 3)
        z = round(data.translation.y, 3) 
        print("Position : x= {}, y= {}, z= {}".format(x,y,z))
    #pipe.stop()
    pass
    

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

def update_map():
    obx =[]
    oby =[]

    for i in range(5):
            print(i)
            current_data= rp.get_data()
            for point in current_data:
                if point[0]==15:
                    obx.append((point[2]*np.sin(point[1]*pi/180))*0.001)
                    oby.append((point[2]*np.cos(point[1]*pi/180))*0.001)
    return [obx,oby]

def ObstacleProche():
    obx =[]
    oby =[]
    

    Obstdistance = 1

    for i in range(1):
            print(i)
            current_data= rp.get_data()
            for point in current_data:
                if point[0]==15:
                    obx.append((point[2]*np.sin(point[1]*pi/180))*0.001)
                    oby.append((point[2]*np.cos(point[1]*pi/180))*0.001)
                    if(point[1]>=300 or point[1]<=60):
                        print(point[2])
                        if ((point[2]*0.001)<=Obstdistance):
                            print("true")
                            return [True,obx,oby]
    return [False,obx,oby]
 
show_animation = False
move_robot = True

def main():
    print("start !!!")

    pipe = cm.initcam()

    if move_robot:
        arduino = serial.Serial(port='COM12', baudrate=115200, timeout=.1)
        write_read("#B",arduino)
        write_read("V",arduino)
        write_read("#B",arduino)
        write_read("R",arduino)
        write_read("r",arduino)

    get_pose(pipe) #initial pose
    '''
    tmp = ObstacleProche()

    ox, oy = [], []
    ox, oy = tmp[1], tmp[2]

    plt.plot(ox, oy, ".k")
    plt.grid(True)
    plt.axis("equal")
    plt.ion()
    plt.show()

    i_tmp = 0

    for i in range(13):
        tmp = ObstacleProche()

        i_tmp = i_tmp + 300
        i_str= str(i_tmp)
        ox, oy = [], []
        ox, oy = tmp[1], tmp[2]

        plt.clf()
        plt.plot(ox, oy, ".k")
        plt.grid(True)
        plt.axis("equal")
        plt.draw()
        plt.pause(0.05)

        if(not(tmp[0])):
            write_read("0"+ "xAt/"+ i_str + "yAt", arduino)
        else:
            break
    pass
'''
    write_read("0"+ "xAt/"+ "400" + "yAt", arduino)
    time.sleep(3)
    get_pose(pipe) 
    
    '''
    write_read("-90zAt", arduino)
    time.sleep(3)
    get_pose(pipe)


    write_read("200"+ "xAt/"+ "700" + "yAt", arduino)
    time.sleep(3)
    get_pose(pipe)

    write_read("0"+ "xAt/"+ "700" + "yAt", arduino)
    time.sleep(3)
    get_pose(pipe)

    write_read("0zAt", arduino)
    time.sleep(3)
    get_pose(pipe)

    '''

    write_read("0" + "xAt/"+ "0" + "yAt", arduino)
    time.sleep(1)



    print("End !!!")
    pipe.stop()

    """
    ox, oy = [], []
    tmp1 = update_map()
    ox, oy = tmp1[0], tmp1[1]

    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.grid(True)
        plt.axis("equal")
    """


if __name__ == '__main__':
    main()
