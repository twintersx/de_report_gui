# source ~/de_report_gui/venv/bin/activate
# pip3 install -U pip -U setuptools
# pip install pillow opencv-python imageio tkcalendar tkintermapview 
from os import path, getcwd
from sys import exit
from subprocess import Popen, PIPE
from functools import partial
from csv import reader, writer 
from threading import Lock, Thread, Event
from itertools import count, cycle
from tkinter import Tk, Frame, Button, Toplevel, Text, Label, Radiobutton, StringVar, font, messagebox, END, TOP, BOTTOM, LEFT, RIGHT, X, Y
from tkintermapview import TkinterMapView
from tkcalendar import Calendar 
from cv2 import VideoCapture, cvtColor, resize, COLOR_BGR2RGB, INTER_AREA, destroyAllWindows
from imageio import get_writer
from PIL import ImageTk, Image           
from datetime import datetime, date, timedelta      
from time import sleep

inputs = {
    'vehicle_name': 'Bravo',
    'vehicle_vin': '1N4AZ1CP7KC308251',
    'csv_file': 'de_reports.csv',
    'recording_folder': 'recordings',
    'ros_file': 'ros.py'
}

streamFrm = []  # globally pass frames from LiveStream() to RecordGIF() threads
lock = Lock()   # Lock() prevents other threads from accessing a shared variable
event = Event() # global variable needed to stop LiveStream thread from root.mainloop()

#DATE,VEHICLE,VIN,ROAD,LATITUDE,LONGITUDE,RECORDING FILE,DESCRIPTION
with open(inputs['csv_file'], newline='') as csvfile:
    headers = list(reader(csvfile, delimiter=','))[0]
dateIndex = headers.index("DATE")
vehicleIndex = headers.index("VEHICLE")
vinIndex = headers.index("VIN")
roadIndex = headers.index('ROAD')
latIndex = headers.index('LATITUDE')
longIndex = headers.index('LONGITUDE')
recFileIndex = headers.index('RECORDING FILE')
descIndex = headers.index('DESCRIPTION')