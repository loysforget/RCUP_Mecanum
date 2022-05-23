import RPLIDAR_fcts as rp
import CamPoseData_fcts as cm
import math
import matplotlib.pyplot as plt
import numpy as np
from math import pi
import serial
import time
import sys
import tkinter as tk
from tkinter import Spinbox, ttk


#-----------Block algorithm-----------------------------
class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr):
        """
        Initialize grid map for a star planning
        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy, show_animation):
        """
        A star path search
        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]
        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while 1:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node, open_set[o]))
            current = open_set[c_id]

            # show graph
            if not(show_animation):  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(
                                                 0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        """
        calc grid position
        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.min_x = -15
        self.min_y = -15

        self.max_x = 15
        self.max_y = 15
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion

#------------------------------------------------------------------------


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
    return data

def update_map(pipe, angle =0):
    tmp = []
    x, y = [], []

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
                    obx.append(x + (point[2]*np.sin((point[1] - angle)*pi/180))*0.001)
                    oby.append(y + (point[2]*np.cos((point[1] - angle)*pi/180))*0.001)
    return [obx,oby]

def ObstacleProche():
    Obstdistance = 0.65 #0.65
    for i in range(2):
        print(i)
        current_data= rp.get_data()
        for point in current_data:
            if point[0]==15:
                if(point[1]>=270 or point[1]<=90):
                    if ((point[2]*0.001)<=Obstdistance):
                        return True
    return False

    
#Fonction qui actualise le chemin a suivre, c'est elle qui execute l'algorithme de deplacement
def update_path(_ox, _oy, _grid_size, _robot_radius, _sx, _sy, _gx, _gy, show_animation):
    _a_star = AStarPlanner(_ox, _oy, _grid_size, _robot_radius)
    _rx, _ry = _a_star.planning(_sx, _sy, _gx, _gy, show_animation)
    return [_rx,_ry]
#-----------------------------------------------------------

def ObjectifAtteint(rx, ry, gx, gy):
    print("RY IS : {} & RX IS : {}".format(ry, rx))
    return ((abs(gx - rx)< 0.2) and (abs(gy - ry)< 0.2))



def run(grid_size, robot_radius, show_animation, move_robot, ArduinoSerialPort, gx, gy, angle_orientation_finale):
    print(__file__ + " start !!!")

    #POSITION DU LIDAR :
    lidar_avant = False
#-----Bloc 1-------------------------------------------------------------
    pipe = cm.initcam()

    if move_robot:
        arduino = serial.Serial(port=ArduinoSerialPort, baudrate=115200, timeout=.1) 
        write_read("#B",arduino)
        write_read("V",arduino)
        write_read("R",arduino)
        write_read("r",arduino)
    angle_orientation = 0
    rx_old = 0
    ry_old = 0
    sx = 0
    sy = 0
    xr = 0
    yr = 0
    flag = 0
    Obstacle = False
    ox, oy = [], []


    while(True):

#-----Bloc 2-------------------------------------------------------------
        """
        On verifie toujours au début si on est arrivé à distination et on arrète le programme si c'est le cas
        """
        if(not(ObjectifAtteint(xr, yr, gx, gy))):

            tmp1 = []
            tmp1 = update_map(pipe, angle_orientation)
            ox.extend(tmp1[0])
            oy.extend(tmp1[1]) 

            if show_animation:  # pragma: no cover
                plt.plot(ox, oy, ".k")
                plt.plot(sx, sy, "og")
                plt.plot(gx, gy, "xb")
                plt.grid(True)
                plt.axis("equal")
            
            if lidar_avant:
                tmp2 = update_path(ox, oy, grid_size, robot_radius, xr, yr-0.020, gx, gy, show_animation)
            else :
                tmp2 = update_path(ox, oy, grid_size, robot_radius, xr, yr, gx, gy, show_animation)
            rx, ry = [], []
            rx, ry = tmp2[0], tmp2[1]
            
            if show_animation:  # pragma: no cover
                #plt.plot(rx, ry, "-r")
                plt.scatter(rx, ry)
                plt.savefig('C:/Users/Loys Forget/Documents/VSCode/RCUP_Mecanum/RCUP_Mecanum/Code complet/Mecanum/2D_map/figure.png')
                plt.pause(0.001)
                plt.draw()
                plt.pause(1)
                plt.clf()
                
                

#-----Bloc 3-------------------------------------------------------------
            if move_robot:

                #-----Bloc 3.1--------------------------------------------------
                
                #envoie de la commande pour déplacer le robot suivant le chemin trouve
                j=0
                for j in range(len(rx)):
                    rx_new = rx[len(rx) - j - 1]*1000
                    ry_new = ry[len(rx) - j - 1]*1000

                    angle_orientation_old  = angle_orientation 

                    if ((rx_new - rx_old == 0  or abs(rx_new - rx_old) > 0) and ry_new - ry_old > 0):
                        angle_orientation = 0
                    elif(rx_new - rx_old > 0 and ry_new - ry_old == 0):
                        angle_orientation = -90
                    elif((rx_new - rx_old < 0 and ry_new - ry_old == 0)):
                        angle_orientation = 90
                    elif((rx_new - rx_old == 0  or rx_new - rx_old > 0) and ry_new - ry_old < 0):
                        angle_orientation = -180
                    elif(rx_new - rx_old < 0 and ry_new - ry_old < 0):
                        angle_orientation = 180

                    '''
                    Ajouter une fonction qui arrete le robot si un obstacle est proche
                    dans ce cas on reexecute la boucle while
                    '''
                    #-----Bloc 3.2--------------------------------------------------
                    Obstacle  = ObstacleProche()
            
                    if(Obstacle):   
                        if (flag==0):                              #faire 4 pas avant de reverifier s'il y a un nouveau obstacle
                            print("obstacle proche !!")
                            #write_read("4M0W", arduino)
                            flag = 1
                            break 
                        elif(flag==1):
                            print("flag == 1")
                            flag = 2
                        elif(flag==2):
                            print("flag == 2")
                            flag = 3
                        elif(flag==3):
                            print("flag == 3")
                            flag = 4    
                        elif(flag==4):
                            print("flag == 4")
                            flag = 0

                    #-----Bloc 3.3--------------------------------------------------     
                    angle_orientation_str = str(angle_orientation)  #---_tmp

                    if(angle_orientation - angle_orientation_old != 0):
                        write_read(angle_orientation_str + "zAt", arduino)
                        print("Rotation: " + angle_orientation_str + "zAt")
                    
                    if (angle_orientation - angle_orientation_old != 0):
                        if(abs(angle_orientation - angle_orientation_old) >= 180 and (abs(angle_orientation - angle_orientation_old) != 360)):
                            time.sleep(5.5)
                            break    
                        else:
                            time.sleep(3.5)
                            #break
                    rx_old = rx_new
                    ry_old = ry_new

                    rx_c = str(rx_new)
                    ry_c = str(ry_new)

                    write_read(rx_c + "xAt/"+ ry_c + "yAt", arduino)
                    print("Translation : " + rx_c + "xAt/"+ ry_c + "yAt")

                xr = rx_old*0.001
                yr = ry_old*0.001

#-----Bloc 4--------------------------------------------------
        else:
            break

    angle_orientation_finale_str = str(angle_orientation_finale)
    write_read(angle_orientation_finale_str + "zAt", arduino)
    print("Rotation finale: " + angle_orientation_finale_str + "zAt")

    print("")
    print("Objectif atteint !!!")

    pipe.stop()

    pass

def main():
    root = tk.Tk()
    root.title("Robot Mecanum Console")

    ### ---------- Dimension de la fenetre et centrage ---------- ###

    window_width = 1000
    window_height = 600

    # get the screen dimension
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()

    # find the center point
    center_x = int(screen_width/2 - window_width / 2)
    center_y = int(screen_height/2 - window_height / 2)

    # set the position of the window to the center of the screen
    #root.geometry(f'{window_width}x{window_height}+{center_x}+{center_y}')

    move_robot = tk.BooleanVar()
    show_animation = tk.BooleanVar()

    ly = ttk.Label(root, text = "Position y")
    lx = ttk.Label(root, text = "Position x")
    langle = ttk.Label(root, text = "Angle de fin")
    lrayon = ttk.Label(root, text = "Rayon de securite")
    lport = ttk.Label(root, text = "PORT ARDUINO")

    Entry_gy = ttk.Entry(root, width=5)
    Entry_gy.insert(0, "0")
    Entry_gx = ttk.Entry(root, width=5)
    Entry_gx.insert(0, "0")
    Entry_angle_orientation_finale = ttk.Entry(root, width=5)
    Entry_angle_orientation_finale.insert(0, "0")
    Entry_rayon = ttk.Entry(root, width=5)
    Entry_rayon.insert(0, "0.5")
    port_arduino = ttk.Entry(root, width=7)

    Checkbutton_deplacement = ttk.Checkbutton(root, text = "Déplacement", variable=move_robot, onvalue=True, offvalue=False)
    Checkbutton_affichage = ttk.Checkbutton(root, text = "Affichage", variable=show_animation, onvalue=True, offvalue=False)

    
    img = tk.PhotoImage(file = 'C:/Users/Loys Forget/Documents/VSCode/RCUP_Mecanum/RCUP_Mecanum/Code complet/Mecanum/2D_map/figure.png')
    img1 = img.subsample(1, 1)

    def quick_scan():
        pipe = cm.initcam()
        ox, oy = [], []
        tmp1 = []
        tmp1 = update_map(pipe)
        ox.extend(tmp1[0])
        oy.extend(tmp1[1]) 
        plt.plot(ox, oy, ".k")
        plt.grid(True)
        plt.axis("equal")
        plt.savefig('C:/Users/Loys Forget/Documents/VSCode/RCUP_Mecanum/RCUP_Mecanum/Code complet/Mecanum/2D_map/figure.png')

    def button_run():
        gy = float(Entry_gy.get())
        gx = float(Entry_gx.get())
        robot_radius = float(Entry_rayon.get())
        angle_orientation_finale = float(Entry_angle_orientation_finale.get())
        ArduinoSerialPort = port_arduino.get()
        print(gy, gx, " ", robot_radius, angle_orientation_finale, ArduinoSerialPort, move_robot, show_animation)
        run(grid_size=0.25, robot_radius=robot_radius, show_animation=show_animation, move_robot=move_robot, ArduinoSerialPort=ArduinoSerialPort, gx=gx, gy=gy, angle_orientation_finale=angle_orientation_finale)

    Button_run = ttk.Button(root, text="START", command=button_run)
    Button_scan = ttk.Button(root, text="SCAN", command=quick_scan)

    ly.grid(row = 0, column = 0, pady = 2)
    lx.grid(row = 1, column = 0, pady = 2)
    Entry_gy.grid(row = 0, column = 1, stick = 'nsew')
    Entry_gx.grid(row = 1, column = 1, stick = 'nsew')

    langle.grid(row = 3, column = 0, pady = 2)
    lrayon.grid(row = 4, column = 0, pady = 2)
    Entry_angle_orientation_finale.grid(row = 3, column = 1, stick = 'nsew')
    Entry_rayon.grid(row = 4, column = 1, stick = 'nsew')
    Checkbutton_affichage.grid(row = 5, column = 0, stick = 'nsew')
    Checkbutton_deplacement.grid(row = 6, column = 0, stick = 'nsew')
    lport.grid(row = 7, column = 0, pady = 2)
    port_arduino.grid(row = 7, column = 1, stick = 'nsew')
    Button_run.grid(row = 9, column = 0, stick = 'nsew')
    Button_scan.grid(row = 9, column = 1, stick = 'nsew')

    tk.Label(root, image = img1).grid(row = 0, column = 3, columnspan = 9, rowspan = 20, padx = 5, pady = 5)

    #rcup_finale.parametrage(0.25, 0.5, True, True, 'COM12', -3.2, 0, 0)

    root.mainloop()





if __name__ == '__main__':
    main()
