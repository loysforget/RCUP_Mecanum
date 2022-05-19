# Importing Libraries
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

    return data

arduino = serial.Serial(port='COM12', baudrate=115200, timeout=.1) #Faut préciser le port sur lequel est branchée la carte Arduino

write_read("#B", arduino)
write_read("V", arduino)
write_read("V", arduino)


