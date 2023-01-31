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
from itertools import count, cycle

streamFrm = []
lock = threading.Lock()

class LiveStream():
    def __init__(self):
        ts = threading.Thread(target=self.stream_start, daemon=True)
        ts.start()

    def stream_start(self):
        global streamFrm
        cap = cv2.VideoCapture(0)
        while True:
            ret, frame = cap.read()
            with lock:
                streamFrm.append((datetime.now(), frame))
            sleep(0.25)  # fps

class RootWindow(tk.Frame):
# ---------- INITIALIZATION --------- #
    def __init__(self, parent):
        tk.Frame.__init__(self, parent)
        self.parent = parent
        self.pad = 5
        self.initHeaders()
        self.initLogGUI()

    def initHeaders(self):
        with open('reports.csv', newline='') as csvfile:
            self.headers = list(csv.reader(csvfile, delimiter=','))[0]
        self.dateIndex = self.headers.index("DATE")
        self.vinIndex = self.headers.index("VIN")
        self.roadIndex = self.headers.index('ROAD')
        self.latIndex = self.headers.index('LATITUDE')
        self.longIndex = self.headers.index('LONGITUDE')
        self.recFileIndex = self.headers.index('RECORDING FILE')
        self.descIndex = self.headers.index('DESCRIPTION')

    def initLogGUI(self):
        self.logFrame = tk.Frame(self.parent)
        self.logFrame.pack(fill='both')
        
        helv36 = tkFont.Font(family='Helvetica', size=36, weight='bold')
        self.logButton = tk.Button(self.logFrame, height=10, width=20, text="RECORD\nDISENGAGMENT", command=self.logDEvent, bg='green', font=helv36)    #
        self.logButton.pack(side=tk.TOP, fill='both', padx=self.pad, pady=self.pad)

        helv10 = tkFont.Font(family='Helvetica', size=20, weight='bold')
        endDriveButton = tk.Button(self.logFrame, text="END DRIVE", command=self.initReportGUI, bg='red', font=helv10)
        endDriveButton.pack(side=tk.BOTTOM, fill='both', padx=self.pad, pady=self.pad)

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

        self.saveFrame = tk.Frame(self.controlFrame)
        self.saveFrame.pack(fill=tk.X, side=tk.BOTTOM, padx=self.pad, pady=self.pad)

        self.gifWindow = tk.Toplevel()
        self.gifWindow.withdraw()
        self.gifWindow.attributes('-topmost',True)

    def initMapWidget(self):
        self.map_widget = TkinterMapView(self.mapFrame, width=1000, height=1000)
# ---------- END INITIALIZATION --------- #

# ---------- RECORD WINDOW ---------- #
    def recordGIF(self):
        global streamFrm
        self.recordTime = datetime.now()
        self.gifFileName = self.recordTime.strftime("%m%d%Y_%H%M%S") + '.gif'
        tMinus10 = self.recordTime - timedelta(seconds=10)
        sleep(10)
        gifFrames = []
        for data in streamFrm:
            if data[0] >= tMinus10:
                gifFrames.append(data[1])

        with imageio.get_writer(f'recordings\{self.gifFileName}') as writer:
            for f in gifFrames:
                rgb_frame = cv2.cvtColor(f, cv2.COLOR_BGR2RGB)
                writer.append_data(rgb_frame)
            with lock:
                streamFrm = []

    def captureGPS(self):
        #ROS
        self.latitude = ''
        self.longitude = ''
        pass

    def writeNewCSVRow(self):
        newLog = [None] * len(self.headers)
        newLog[self.dateIndex] = self.recordTime.strftime('%m/%d/%Y')
        newLog[self.vinIndex] = '1N4AZ1CP7KC308251'
        newLog[self.roadIndex] = ''
        newLog[self.latIndex] = self.latitude
        newLog[self.longIndex] = self.longitude
        newLog[self.recFileIndex] = self.gifFileName
        newLog[self.descIndex] = ''

        with open('reports.csv', 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(newLog)

    def logDEvent(self):
        self.recordGIF()
        self.captureGPS()
        self.writeNewCSVRow()
# ---------- END RECORD WINDOW ---------- #
    
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
    
# ---------- BUTTON FUNCTIONALITY ---------- #
    def onDateChangeClick(self):
        self.setDateReport()
        self.initMapPosition()
        self.initReportButtons()

    def setDateReport(self):
        self.report = []
        with open('reports.csv', newline='') as csvfile:
            reader = list(csv.reader(csvfile, delimiter=','))
            for row in reader[1:]:
                report_date = datetime.strptime(row[self.dateIndex], '%m/%d/%Y')
                if self.initialDate.day <= report_date.day <= self.finalDate.day:
                    self.report.append(row)

    def changeMapPosition(self, lat, long, zoom):
        self.map_widget.set_position(lat, long)
        self.map_widget.set_zoom(zoom)
        self.map_widget.pack(fill='both')
    
    def initMapPosition(self):
        # at least one report
        if len(self.report) >= 1:
            lats, longs = [], []
            for row in self.report:
                lats.append(float(row[self.latIndex]))
                longs.append(float(row[self.longIndex]))
            lat = sum(lats) / len(lats)
            long = sum(longs) / len(longs)

        # no reports, set to lab center
        elif len(self.report) == 0:
            lat, long = 37.376774, -121.989967

        self.changeMapPosition(lat, long, 12)

    def clickMarker(self, marker):
        if marker.image_hidden is True:
            marker.hide_image(False)
        else:
            marker.hide_image(True)

    def highlightButton(self, reportButton):
        for button in self.buttons:
            if button == reportButton:
                reportButton.config(bg='red')
            else:
                button.config(bg='white')

    def displayGif(self, recFile, textOnButton):
        self.clearWidgets(self.gifWindow, 'destroy')
        self.gifWindow.title(textOnButton)

        gifPath = os.path.join(os.getcwd(), 'recordings', recFile)
        self.gifFrame = tk.Label(self.gifWindow)
        self.gifFrame.pack()

        if isinstance(gifPath, str):
            im = Image.open(gifPath)
        frames = []
        self.frames = cycle(frames)
        self.delay = im.info['duration']

        try:
            for i in count(1):
                frames.append(ImageTk.PhotoImage(im.copy()))
                im.seek(i)
        except EOFError:
            pass

        self.gifWindow.deiconify()
        self.next_frame()
    
    def next_frame(self):
        if self.frames:
            self.gifFrame.config(image=next(self.frames))
            self.gifFrame.after(self.delay, self.next_frame)

    def disengagmentFocus(self, i, lat, long, zoom, recFile, descBox, title, marker):
        self.changeMapPosition(lat, long, zoom)
        self.initInputWidgets(descBox)
        self.highlightButton(self.buttons[i])
        self.clickMarker(self.markers[i])
        self.displayGif(recFile, title)

    def initReportButtons(self):
        self.clearWidgets(self.reportButtonFrame, 'destroy') 
        self.clearWidgets(self.userInputFrame, 'destroy')
        self.map_widget.delete_all_marker()   
        self.buttons = []
        self.markers = []
        for i, row in enumerate(self.report):
            lat, long = float(row[self.latIndex]), float(row[self.longIndex])

            recFile = row[self.recFileIndex]
            recDateObj = datetime.strptime(recFile.split('.')[0], '%m%d%Y_%H%M%S')
            formatRecDate = datetime.strftime(recDateObj, '%m/%d/%Y %H:%M:%S')
            title = f"{str(formatRecDate)} \n({lat}, {long})"
              
            descBox = tk.Text(self.userInputFrame, width=5, height=5) 

            marker = self.map_widget.set_marker(lat, long, command=partial(self.disengagmentFocus, i, lat, long, 15, recFile, descBox, title))
            self.markers.append(marker)

            reportButton = tk.Button(self.reportButtonFrame, text=title, command=partial(self.disengagmentFocus,i, lat, long, 15, recFile, descBox, title, None))
            reportButton.pack(fill='both', padx=self.pad, pady=self.pad) 
            self.buttons.append(reportButton)

    def clearWidgets(self, frame, action):
        widgets = frame.winfo_children()
        for w in widgets:
            if action == 'destroy':
                w.destroy()
            elif action == 'hide':
                w.pack_forget()

    def initInputWidgets(self, descBox):
        self.clearWidgets(self.userInputFrame, 'hide')  #all widgets are being removed so when pack is called, it is removing the box and not attached
        tk.Label(self.userInputFrame, text="DESCRIPTION OF REASON FOR DISENGAGMENT").pack()
        descBox.config(highlightthickness=3, highlightbackground = "red", highlightcolor='red')
        descBox.pack(fill=tk.X)
        tk.Label(self.userInputFrame, text="SELECT ROAD TYPE: ").pack(side=tk.LEFT)
        self.radioButton = tk.Radiobutton(self.userInputFrame, text='Highway', value='Highway').pack(side=tk.RIGHT)
        self.radioButton = tk.Radiobutton(self.userInputFrame, text='Street', value='Street').pack(side=tk.RIGHT)

    def saveUserInputs(self):
        pass


    def initSave(self):
        tk.Button(self.saveFrame, text='SAVE', command=self.saveUserInputs).pack(fill=tk.X)
        self.saveText = tk.Text(self.saveFrame, width=5, height=5)
        self.saveText.pack(fill=tk.X)
        self.saveText.insert('1.0', "A log of the last save will be shown here") #1.0 line 1 char 0
# ---------- END BUTTON FUNCTIONALITY ---------- #
    
if __name__ == "__main__":
    LiveStream().stream_start
    root = tk.Tk()
    RootWindow(root).pack(side='top', fill='both', expand=True)
    root.mainloop()