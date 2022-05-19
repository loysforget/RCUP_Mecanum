import tkinter as tk
import numpy as np
import matplotlib.pyplot as plt
from tkinter import ttk
import scan_rapide_detect_obst_rapide_finale
import sys
import os


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
root.geometry(f'{window_width}x{window_height}+{center_x}+{center_y}')

root.columnconfigure(2, 1)
root.rowconfigure(6, 1)

ttk.Label(root, text = "TEST LABEL").pack()

os.system("scan_rapide_detect_obst_rapide_finale.py 0.25 0.5 true true COM12 -3.2 0 0")


root.mainloop()