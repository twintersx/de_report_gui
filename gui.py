import tkinter as tk
from tkinter import font as tkFont  # for convenience
from tkcalendar import Calendar # pip install tkcalendar
from datetime import datetime, date, timedelta
from tkintermapview import TkinterMapView  # pip install tkintermapview
import csv
from functools import partial
import cv2, imageio
from PIL import ImageTk, Image  # pip install pillow
import os
import threading
from time import sleep

frames = []
lock = threading.Lock()

class StreamRecorder():
    def __init__(self):
        t = threading.Thread(target=self.threadStream, daemon=True)
        t.start()

    def threadStream(self):
        global frames
        cap = cv2.VideoCapture(0)
        while True:
            ret, frame = cap.read()
            with lock:
                frames.append((datetime.now(), frame))
            sleep(0.25)  # fps

    def captureGPS():
        #ROS
        pass

    def logToCSV():
        pass

class RootWindow(tk.Frame):
# ---------- INITIALIZATION --------- #
    def __init__(self, parent):
        tk.Frame.__init__(self, parent)
        self.parent = parent
        self.pad = 5
        self.initLogGUI()

    def initLogGUI(self):
        self.logFrame = tk.Frame(self.parent)
        self.logFrame.pack(fill='both')

        helv36 = tkFont.Font(family='Helvetica', size=36, weight='bold')
        self.logButton = tk.Button(self.logFrame, height=10, width=20, text="LOG\nDISENGAGMENT", command=self.saveGif, bg='green', font=helv36)
        self.logButton.pack(side=tk.TOP, fill='both', padx=self.pad, pady=self.pad)

        helv10 = tkFont.Font(family='Helvetica', size=20, weight='bold')
        endDriveButton = tk.Button(self.logFrame, text="END DRIVE", command=self.initReportGUI, bg='red', font=helv10)
        endDriveButton.pack(side=tk.BOTTOM, fill='both', padx=self.pad, pady=self.pad)
    

    def saveGif(self):
        global frames
        
        now = datetime.now()
        gifDate = now.strftime("%d%m%Y_%H%M%S")
        tMinus10 = now - timedelta(seconds=10)
        self.logButton.config(text="RECORDING...", bg="red")
        self.logButton.pack()

        sleep(10)

        gifFrames = []
        for data in frames:
            if data[0] >= tMinus10:
                gifFrames.append(data[1])

        with imageio.get_writer(f'recordings\{gifDate}.gif') as writer:
            for f in gifFrames:
                rgb_frame = cv2.cvtColor(f, cv2.COLOR_BGR2RGB)
                writer.append_data(rgb_frame)
            with lock:
                frames = []

        self.logButton.config(text="LOG\nDISENGAGMENT", bg='green')

    def initReportGUI(self):
        self.logFrame.destroy()
        self.initFrames()
        self.initCalendar()
        self.setDateReport()
        self.initMapWidget()
        self.initMapPosition()
        self.initReportButtons()
        self.initSave()

    def initFrames(self):   
        self.mapFrame = tk.Frame(self.parent)
        self.mapFrame.pack(side=tk.RIGHT, fill='both', padx=self.pad, pady=self.pad)

        self.calWindow = tk.Toplevel()
        self.calWindow.withdraw()

        self.controlFrame = tk.Frame(self.parent)
        self.controlFrame.pack(fill='both', side=tk.LEFT, padx=self.pad, pady=self.pad)

        self.calFrame = tk.Frame(self.controlFrame, height=20)
        self.calFrame.pack(fill=tk.X, side=tk.TOP, padx=self.pad, pady=self.pad)

        self.loadFrame = tk.Frame(self.controlFrame)
        self.loadFrame.pack(fill=tk.X, side=tk.TOP, padx=self.pad, pady=self.pad)

        self.reportButtonFrame = tk.Frame(self.controlFrame)
        self.reportButtonFrame.pack(fill=tk.X, padx=self.pad, pady=self.pad)

        self.userInputFrame = tk.Frame(self.controlFrame)
        self.userInputFrame.pack(fill=tk.X, padx=self.pad, pady=self.pad)

        self.logFrame = tk.Frame(self.controlFrame)
        self.logFrame.pack(fill=tk.X, side=tk.BOTTOM, padx=self.pad, pady=self.pad)

# ---------- END INITIALIZATION --------- #
    
# ---------- CALENDAR ---------- #
    def closeInitialCal(self, event):
        self.initialDate = datetime.strptime(self.cal.get_date(), '%m/%d/%y')
        self.initalDateButton.config(text=self.initialDate.strftime('%m/%d/%Y'))
        self.calWindow.withdraw()

    def closeFinalCal(self, event):
        self.finalDate = datetime.strptime(self.cal.get_date(), '%m/%d/%y')
        self.finalDateButton.config(text=self.finalDate.strftime('%m/%d/%Y'))
        self.calWindow.withdraw()

    def setInitialDate(self):
        self.calWindow.deiconify()
        self.calWindow.bind('<Button-1>', self.closeInitialCal)

    def setFinalDate(self):
        self.calWindow.deiconify()
        self.calWindow.bind('<Button-1>', self.closeFinalCal)

    def initCalendar(self):
        today = date.today()
        self.initialDate, self.finalDate = datetime.today(), datetime.today()
        self.cal = Calendar(self.calWindow, selectmode='day', year=today.year, month=today.month, day=today.day, background='orange', foreground='white', borderwidth=2)
        self.cal.pack()

        tk.Label(self.calFrame, text='DATE RANGE: ').pack(fill='both', side=tk.LEFT)

        self.initalDateButton = tk.Button(self.calFrame, text=f"{today.strftime('%m/%d/%Y')}", command=self.setInitialDate)
        self.initalDateButton.pack(fill='both', side=tk.LEFT) 

        tk.Label(self.calFrame, text='-').pack(side=tk.LEFT)

        self.finalDateButton = tk.Button(self.calFrame, text=f"{today.strftime('%m/%d/%Y')}", command=self.setFinalDate)
        self.finalDateButton.pack(fill='both', side=tk.LEFT) 

        self.loadButton = tk.Button(self.loadFrame, text="LOAD", command=self.onDateChangeClick)
        self.loadButton.pack(fill=tk.X, side=tk.BOTTOM) 
# ---------- END CALENDAR ---------- #
    
    def onDateChangeClick(self):
        self.setDateReport()
        self.initMapPosition()
        self.initReportButtons()

    def setDateReport(self):
        self.report = []
        with open('reports.csv', newline='') as csvfile:
            reader = list(csv.reader(csvfile, delimiter=','))
            self.header = reader[0]
            date_index = self.header.index("DATE")
            for row in reader[1:]:
                report_date = datetime.strptime(row[date_index], '%m/%d/%Y')
                if self.initialDate.day <= report_date.day <= self.finalDate.day:
                    self.report.append(row)

    def initMapWidget(self):
        self.map_widget = TkinterMapView(self.mapFrame, width=1000, height=1000)
        self.lat_index = self.header.index("LATITUDE")
        self.long_index = self.header.index("LONGITUDE")
        self.rec_file_index = self.header.index("RECORDING FILE")

    def changeMapPosition(self, lat, long, zoom):
        self.map_widget.set_position(lat, long)
        self.map_widget.set_zoom(zoom)
        self.map_widget.pack(fill='both')
    
    def initMapPosition(self):
        # at least one report
        if len(self.report) >= 1:
            lats, longs = [], []
            for row in self.report:
                lats.append(float(row[self.lat_index]))
                longs.append(float(row[self.long_index]))
            lat = sum(lats) / len(lats)
            long = sum(longs) / len(longs)

        # no reports, set to lab center
        elif len(self.report) == 0:
            lat, long = 37.376774, -121.989967

        self.changeMapPosition(lat, long, 12)
    
    def markerFocus(self, lat, long, zoom, marker):
        self.changeMapPosition(lat, long, zoom)
        self.clickMarker(marker)
        self.initInputWidgets()

    def clickMarker(self, marker):
        if marker.image_hidden is True:
            marker.hide_image(False)
        else:
            marker.hide_image(True)

    def initReportButtons(self):
        self.clearWidgets(self.reportButtonFrame) 
        self.clearWidgets(self.userInputFrame)
        self.map_widget.delete_all_marker()   
        for row in self.report:
            lat, long = float(row[self.lat_index]), float(row[self.long_index])
            recFile = row[self.rec_file_index]
            recDateObj = datetime.strptime(recFile.split('.')[0], '%m%d%Y_%H%M%S')
            formatRecDate = datetime.strftime(recDateObj, '%m/%d/%Y %H:%M:%S')

            text = f"{str(formatRecDate)}\n({lat}, {long})" 
            gif = ImageTk.PhotoImage(Image.open(os.path.join(os.getcwd(), 'recordings', recFile)).resize((450, 375)))
            marker = self.map_widget.set_marker(lat, long, image=gif, command=self.clickMarker)
            marker.hide_image(True)
            reportButton = tk.Button(self.reportButtonFrame, text=text, command=partial(self.markerFocus, lat, long, 15, marker))
            reportButton.pack(fill='both', padx=self.pad, pady=self.pad) 

    def clearWidgets(self, frame):
        widgets = frame.winfo_children()
        for w in widgets:
            w.destroy()

    def initInputWidgets(self):
        self.clearWidgets(self.userInputFrame)
        tk.Label(self.userInputFrame, text="DESCRIPTION OF REASON FOR DISENGAGMENT").pack()
        tk.Text(self.userInputFrame, width=5, height=5).pack(fill=tk.X)
        tk.Label(self.userInputFrame, text="SELECT ROAD TYPE: ").pack(side=tk.LEFT)
        roadRadio = tk.Radiobutton(self.userInputFrame, text='Highway', value='Highway').pack(side=tk.RIGHT)
        roadRadio = tk.Radiobutton(self.userInputFrame, text='Street', value='Street').pack(side=tk.RIGHT)

    def initSave(self):
        tk.Button(self.logFrame, text='SAVE').pack(fill=tk.X)
        self.saveText = tk.Text(self.logFrame, width=5, height=5)
        self.saveText.pack(fill=tk.X)
        self.saveText.insert('1.0', "A log of the last save will be shown here") #1.0 line 1 char 0
            
if __name__ == "__main__":
    StreamRecorder().threadStream
    root = tk.Tk()
    RootWindow(root).pack(side='top', fill='both', expand=True)
    root.mainloop()