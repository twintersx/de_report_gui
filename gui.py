import csv, os
import tkinter as tk
from tkinter import messagebox
from tkinter import font as tkFont 
from tkcalendar import Calendar # pip install tkcalendar
from PIL import ImageTk, Image  # pip install pillow
from datetime import datetime, date, timedelta
from tkintermapview import TkinterMapView  # pip install tkintermapview
from functools import partial
import cv2 # pip install opencv-python
import imageio  # pip install imageio
from threading import Lock, Thread, Event
from time import sleep
from itertools import count, cycle

streamFrm = []
lock = Lock()
event = Event()

class LiveStream():
    def __init__(self):
        ts = Thread(target=self.stream_start, daemon=True)
        ts.start()

    def stream_start(self):
        global streamFrm
        cap = cv2.VideoCapture(0)
        while True:
            ret, frame = cap.read()
            with lock:
                streamFrm.append((datetime.now(), frame))
            sleep(0.25)  # fps
            
            if len(streamFrm) > 1200:
                streamFrm = streamFrm[1100:]

            if event.is_set():
                break

        cap.release()               # Close the window / Release webcam
        cv2.destroyAllWindows()     # De-allocate any associated memory usage
        return

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
        self.logButton = tk.Button(self.logFrame, height=10, width=20, text="RECORD\nDISENGAGMENT", command=self.logDEvent, bg='green', font=helv36) 
        self.logButton.pack(fill='both', padx=self.pad, pady=self.pad)

        helv10 = tkFont.Font(family='Helvetica', size=20, weight='bold')
        endDriveButton = tk.Button(self.logFrame, text="END DRIVE", command=self.initReportGUI, bg='red', font=helv10)
        endDriveButton.pack(fill='both', padx=self.pad, pady=self.pad)

    def initReportGUI(self):
        event.set()
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

        # --- Top Level Window Initialization --- #
        self.calWindow = tk.Toplevel()
        self.calWindow.protocol("WM_DELETE_WINDOW", self.calWindow.withdraw)
        self.calWindow.attributes('-topmost', True)
        self.calWindow.withdraw()
        self.calWindow.resizable(False, False)

        self.gifWindow = tk.Toplevel()
        self.gifWindow.protocol("WM_DELETE_WINDOW", self.gifWindow.withdraw)
        self.gifWindow.withdraw()
        self.gifWindow.attributes('-topmost', True)
        self.gifWindow.resizable(False, False)

    def initMapWidget(self):
        self.map_widget = TkinterMapView(self.mapFrame, width=1000, height=1000)

    def placeWindowRelRoot(self, window, dx, dy):
        x = self.parent.winfo_x()
        y = self.parent.winfo_y()
        window.geometry("+%d+%d" % (x + dx, y + dy))
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
        if self.cal.get_date() != self.initDate:
            self.initialDate = datetime.strptime(self.cal.get_date(), '%m/%d/%y')
            self.initalDateButton.config(text=self.initialDate.strftime('%m/%d/%Y'))
            self.calWindow.withdraw()

    def closeFinalCal(self, event):
        if self.cal.get_date() != self.initDate:
            self.finalDate = datetime.strptime(self.cal.get_date(), '%m/%d/%y')
            self.finalDateButton.config(text=self.finalDate.strftime('%m/%d/%Y'))
            self.calWindow.withdraw()

    def setInitialDate(self):
        self.placeWindowRelRoot(self.calWindow, 0, 110)
        self.calWindow.title("Set Inital Date")
        self.calWindow.deiconify()
        self.initDate = self.cal.get_date()
        self.calWindow.bind('<Button-1>', self.closeInitialCal)

    def setFinalDate(self):
        self.placeWindowRelRoot(self.calWindow, 0, 110)
        self.calWindow.title("Set Final Date")
        self.calWindow.deiconify()
        self.initDate = self.cal.get_date()
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

        self.loadButton = tk.Button(self.loadFrame, text="RELOAD", command=self.onReloadClick)
        self.loadButton.pack(fill=tk.X, side=tk.BOTTOM) 
# ---------- END CALENDAR ---------- #
    
# ---------- BUTTON FUNCTIONALITY ---------- #
    def onReloadClick(self):
        self.saveUserInputs()
        self.setDateReport()
        self.initMapPosition()
        self.initReportButtons()

    def setDateReport(self):
        self.dateRangeReports = []
        with open('reports.csv', newline='') as csvfile:
            reader = list(csv.reader(csvfile, delimiter=','))
            for row in reader[1:]:
                report_date = datetime.strptime(row[self.dateIndex], '%m/%d/%Y')
                if self.initialDate <= report_date <= self.finalDate:
                    self.dateRangeReports.append(row)

    def changeMapPosition(self, i):
        self.map_widget.set_zoom(int(self.map_widget.zoom)) #required
        self.map_widget.set_position(
            self.attributes[i]['lat'], 
            self.attributes[i]['long'])
        self.map_widget.set_zoom(self.attributes[i]['zoom'])
    
    def initMapPosition(self):
        # at least one report
        if len(self.dateRangeReports) >= 1:
            lats, longs = [], []
            for row in self.dateRangeReports:
                lats.append(float(row[self.latIndex]))
                longs.append(float(row[self.longIndex]))
            lat = sum(lats) / len(lats)
            long = sum(longs) / len(longs)

        # no reports, set to lab center
        elif len(self.dateRangeReports) == 0:
            lat, long = 37.376774, -121.989967

        self.map_widget.set_position(lat, long)
        self.map_widget.set_zoom(12)
        self.map_widget.pack(fill='both')

    def clickMarker(self, i):
        if self.attributes[i]['marker'].image_hidden is True:
            self.attributes[i]['marker'].hide_image(False)
        else:
            self.attributes[i]['marker'].hide_image(True)

    def highlightButton(self, i):
        buttons = [*range(0, len(self.attributes))]
        for b in buttons:
            if b == i:
                self.attributes[i]['button'].config(bg='red')
                self.attributes[i]['marker'].delete()
                self.attributes[i]['marker'] = self.map_widget.set_marker(
                    self.attributes[i]['lat'], 
                    self.attributes[i]['long'], 
                    marker_color_circle = 'dark red', 
                    marker_color_outside = 'red', 
                    command=partial(self.disengagmentFocus, i))

            else:
                self.attributes[b]['button'].config(bg='white')
                self.attributes[b]['marker'].delete()
                self.attributes[b]['marker'] = self.map_widget.set_marker(
                    self.attributes[b]['lat'], 
                    self.attributes[b]['long'], 
                    marker_color_circle = 'dark green', 
                    marker_color_outside = 'green', 
                    command=partial(self.disengagmentFocus, b))

    def displayGif(self, i):
        self.clearWidgets(self.gifWindow, 'destroy')
        self.placeWindowRelRoot(self.gifWindow, 550, 0)
        self.gifWindow.title(self.attributes[i]['title'])

        gifPath = os.path.join(os.getcwd(), 'recordings', self.attributes[i]['gifFile'])
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

    def disengagmentFocus(self, i, marker):
        self.changeMapPosition(i)
        self.initInputWidgets(i)
        self.highlightButton(i)
        self.clickMarker(i)
        self.displayGif(i)

    def initReportButtons(self):
        self.clearWidgets(self.reportButtonFrame, 'destroy') 
        self.clearWidgets(self.userInputFrame, 'destroy')
        self.gifWindow.withdraw()
        self.map_widget.delete_all_marker()  

        helv10 = tkFont.Font(family='Helvetica', size=12, weight='bold')
        tk.Label(self.reportButtonFrame, text="DISENGAGMENT LIST", font=helv10).pack(pady=(20, 0))

        self.attributes = []
        for i, row in enumerate(self.dateRangeReports):
            lat, long = float(row[self.latIndex]), float(row[self.longIndex])

            gifFile = row[self.recFileIndex]
            recDateObj = datetime.strptime(gifFile.split('.')[0], '%m%d%Y_%H%M%S')
            formatRecDate = datetime.strftime(recDateObj, '%m/%d/%Y %H:%M:%S')
            title = f"{str(formatRecDate)} \n({lat}, {long})"
              
            descBox = tk.Text(self.userInputFrame, width=5, height=5) 
            descBox.insert('1.0', row[self.descIndex])
            road_type = tk.StringVar(value=row[self.roadIndex])
            radio1 = tk.Radiobutton(self.userInputFrame, text='Highway', value='Highway', variable=road_type)
            radio2 = tk.Radiobutton(self.userInputFrame, text='Street', value='Street', variable=road_type)

            marker = self.map_widget.set_marker(lat, long, marker_color_circle = 'dark green', marker_color_outside = 'green', command=partial(self.disengagmentFocus, i),)

            reportButton = tk.Button(self.reportButtonFrame, text=title, command=partial(self.disengagmentFocus, i, None))
            reportButton.pack(fill='both', padx=self.pad, pady=self.pad) 

            attrib_dict = {
                'i': i,
                'button': reportButton,
                'marker': marker,
                'lat': lat,
                'long': long,
                'zoom': 15,
                'gifFile': gifFile,
                'descBox': descBox,
                'title': title,
                'road_type': road_type,
                'radio1': radio1,
                'radio2': radio2
            }

            self.attributes.append(attrib_dict)

    def clearWidgets(self, frame, action):
        widgets = frame.winfo_children()
        for w in widgets:
            if action == 'destroy':
                w.destroy()
            elif action == 'hide':
                w.pack_forget()

    def initInputWidgets(self, i):
        self.clearWidgets(self.userInputFrame, 'hide') 

        tk.Label(self.userInputFrame, text="DESCRIPTION OF REASON FOR DISENGAGMENT").pack()
        self.attributes[i]['descBox'].config(highlightthickness=3, highlightbackground = "red", highlightcolor='red')
        self.attributes[i]['descBox'].pack(fill=tk.X)

        tk.Label(self.userInputFrame, text="SELECT ROAD TYPE: ").pack(side=tk.LEFT)
        self.attributes[i]['radio1'].pack(side=tk.RIGHT)
        self.attributes[i]['radio2'].pack(side=tk.RIGHT)

    def saveUserInputs(self):
        with open('reports.csv', newline='') as csvfile:
            all_reports = list(csv.reader(csvfile, delimiter=','))

        with open('reports.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile) 

            for row in all_reports[1:]:   # skip headers
                for i, attrib in enumerate(self.attributes):                       
                    if attrib['gifFile'] == row[self.recFileIndex]:

                        road_type = attrib['road_type'].get()
                        row[self.roadIndex] = road_type
                        
                        description = attrib['descBox'].get("1.0", tk.END) 
                        description = description.replace('\n', '')
                        row[self.descIndex] = description    

                        """if i <= len(self.dateRangeReports) - 1:   
                            self.dateRangeReports[i][self.roadIndex] = road_type     
                            self.dateRangeReports[i][self.descIndex] = description  """   

                        break

            writer.writerows(all_reports)

        self.saveText.insert('1.0', f"SAVED: {datetime.now().strftime('%H:%M:%S') }\n") #1.0 line 1 char 0

    def initSave(self):
        tk.Button(self.saveFrame, text='SAVE', command=self.saveUserInputs).pack(fill=tk.X)
        self.saveText = tk.Text(self.saveFrame, width=5, height=5)
        self.saveText.pack(fill=tk.X)
        tk.Button(self.saveFrame, text='EXIT GUI', command=self.onUserClose).pack(fill=tk.X)

    def onUserClose(self): 
        self.saveUserInputs()
        if messagebox.askokcancel('Disengagment GUI 2.0', "Your work has auto-saved!\nAre you sure you want to quit?", parent=self.gifWindow):
            self.parent.destroy()

# ---------- END BUTTON FUNCTIONALITY ---------- #

def main():
    root = tk.Tk()
    root.resizable(False, False)
    root.title('Disengagment GUI 2.0')
    #root.attributes('-fullscreen', True)

    rw = RootWindow(root)
    rw.pack(side='top', fill='both', expand=True)
    root.protocol("WM_DELETE_WINDOW", rw.onUserClose)

    LiveStream().stream_start

    root.mainloop()

if __name__ == "__main__":
    main()