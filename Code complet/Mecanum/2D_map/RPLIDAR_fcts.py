import numpy as np
import rplidar
from rplidar import RPLidar

def get_data():
    """
    Obtention des données du Lidar
        input:

        output:
            scan: list (a revoir)
    """
    lidar = RPLidar('COM4') #Faut spécifier le port auquel est connecter le Lidar ici
    lidar.motor_speed = rplidar.MAX_MOTOR_PWM
    for scan in lidar.iter_scans(min_len=100, max_buf_meas=False):
        break
    lidar.stop()
    return scan


