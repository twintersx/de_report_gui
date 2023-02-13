from dependencies import *

class RecordGif():
    def __init__(self, logButton):
        # Recording of GIF runs in thread to allow multiple recordings in the same timeframe, 
        # additional button functionality and ability to change text/color of button while recording. 
        Thread(target=self.logDEvent).start()
        self.logButton = logButton

    def logDEvent(self):
        # Main function structure of recording thread
        self.recordTime = datetime.now()
        self.captureGPS()
        self.recordGIF()
        self.logButton.config(text="RECORD\nDISENGAGMENT", bg='green')
        self.saveGIF()
        self.writeNewCSVRow()
        
        return  # needed to terminate thread

    def captureGPS(self):
        # small chance GPS coordinates are not captured from ROS. 
        # retries up to 10 times but breaks early if successfull
        path = path.join(getcwd(), inputs['ros_file'])
        n = 0
        while n < 10:
            try:
                # need to run ROS python script outside of virtual enviorment with python2 (for ROS1)
                # the script is executed in the systems root directory '/'
                # the script terminal output is captured via stdout=subprocess.PIPE
                proc = Popen(['python2', path], cwd='/', stdout=PIPE)

                # the output is captured and the coordinates are divided
                output = proc.communicate()[0].decode()
                coordinates = output.split('\n')[0]

                # if 1st row of "coordinates" is not split by unqiue delimiter or a number, an exception is raised and the loop retries
                self.latitude, self.longitude = coordinates.split('abc123xyz')  # this delimiter is set in the de_gui_ros.py script 
                self.latitude = float(self.latitude)
                self.longitude = float(self.longitude)
            except:
                n += 1
                continue 

            # No exception was raised, continue to recordGIF()
            return
        
        print("\nNot able to capture GPS coordinates from ROS.\nHas a GPS LOCK been established?\n")

    def recordGIF(self):
        global streamFrm # used to access a variable across threads
        self.gifFileName = self.recordTime.strftime("%m%d%Y_%H%M%S") + '.gif'
        tMinus15 = self.recordTime - timedelta(seconds=10)
        
        sleep(10)   # wait until LiveStream captures more stream frames

        # gets all frames from tMinus# to newest recorded frame
        self.gifFrames = []
        for data in streamFrm:
            if data[0] >= tMinus15:
                self.gifFrames.append(data[1])

    def saveGIF(self):
        recording_folder = path.join(getcwd(), inputs['recording_folder'], self.gifFileName)
        # Saves group of frames as a .gif in "recordings" folder
        with get_writer(recording_folder) as io_writer:
            for f in self.gifFrames:
                rgb_frame = cvtColor(f, COLOR_BGR2RGB)  # imageio needs RGB format
                
                # resize image to view easier in reporting window
                scale_percent = 150
                width = int(rgb_frame.shape[1] * scale_percent / 100)
                height = int(rgb_frame.shape[0] * scale_percent / 100)
                resized = resize(rgb_frame, (width, height), interpolation = INTER_AREA)

                io_writer.append_data(resized)

    def writeNewCSVRow(self):
        newLog = [None] * len(headers)
        newLog[dateIndex] = self.recordTime.strftime('%m/%d/%Y')
        newLog[vehicleIndex] = inputs['vehicle_name']
        newLog[vinIndex] = inputs['vehicle_vin']
        newLog[roadIndex] = ''
        newLog[latIndex] = self.latitude
        newLog[longIndex] = (-abs(self.longitude)) # force positive value from /gps_state to negative
        newLog[recFileIndex] = self.gifFileName
        newLog[descIndex] = ''

        with open (inputs['csv_file'], newline='') as csvfile:
            reports = list(reader(csvfile, delimiter=','))

        # removes empty lines if csv is manually edited outside of GUI
        for row in list(reports):
            if not row:
                reports.remove(row)

        reports.append(newLog)
        with open(inputs['csv_file'], 'w', newline='') as csvfile:
            csv_writer = writer(csvfile)
            csv_writer.writerows(reports)

class LiveStream():
    def __init__(self):
        Thread(target=self.stream_start).start()

    def stream_start(self):
        global streamFrm

        cap = VideoCapture(0)
        while True:
            ret, frame = cap.read()
            with lock:  
                streamFrm.append((datetime.now(), frame))

            fps, buffer_limit_seconds = 2, 300
            sleep(1 / fps) 

            if len(streamFrm) > fps * buffer_limit_seconds: 
                with lock:
                    # when buffer limit is reached, save only the last 20 seconds 
                    streamFrm = streamFrm[fps * (buffer_limit_seconds - 20):]

            # break from this while loop from root.mainloop() (stop recording once drive is done)
            if event.is_set():
                break

        cap.release()               # Close the window / Release webcam
        destroyAllWindows()     # De-allocate any associated memory usage
        return

class RootWindow(Frame):
# ---------- INITIALIZATION --------- #
    def __init__(self, parent):
        Frame.__init__(self, parent)
        self.pack(fill='both', expand=True)

        self.parent = parent
        self.parent.title('Disengagment GUI 2.0')
        self.parent.resizable(False, False)
        self.parent.protocol("WM_DELETE_WINDOW", self.onUserClose)   # needed to handle root window closing by user

        self.pad = 5
        self.initLogGUI()

    def initLogGUI(self):
        self.logFrame = Frame(self.parent)
        self.logFrame.pack(fill='both')
        
        helv36 = font.Font(family='Helvetica', size=36, weight='bold')
        self.logButton = Button(self.logFrame, height=10, width=20, text="RECORD\nDISENGAGMENT", command=self.recordButtonFunc, bg='green', font=helv36) 
        self.logButton.pack(fill='both', padx=self.pad, side=TOP)

        helv20 = font.Font(family='Helvetica', size=20, weight='bold')
        endDriveButton = Button(self.logFrame, text="END DRIVE", command=self.initReportGUI, bg='red', font=helv20)
        endDriveButton.pack(fill='both', padx=self.pad, pady=self.pad, side=BOTTOM)

    def recordButtonFunc(self):
        self.logButton.config(text='RECORDING...', bg='red')
        RecordGif(self.logButton)

    def initReportGUI(self):
        event.set() # stop recording stream thread
        self.logFrame.destroy()
        self.initFrames()
        self.initCalendar()
        self.setDateReport()
        self.initMapWidget()
        self.initMapPosition()
        self.initReportButtons()
        self.initSaveLog()

    def initFrames(self):   
        self.mapFrame = Frame(self.parent)
        self.mapFrame.pack(side=RIGHT, fill='both', padx=self.pad, pady=self.pad)
        
        self.controlFrame = Frame(self.parent)
        self.controlFrame.pack(fill='both', side=LEFT, padx=self.pad, pady=self.pad)

        self.calFrame = Frame(self.controlFrame, height=20)
        self.calFrame.pack(fill=X, side=TOP, padx=self.pad, pady=self.pad)

        self.loadFrame = Frame(self.controlFrame)
        self.loadFrame.pack(fill=X, side=TOP, padx=self.pad, pady=self.pad)

        self.reportButtonFrame = Frame(self.controlFrame)
        self.reportButtonFrame.pack(fill=X, padx=self.pad, pady=self.pad)

        self.userInputFrame = Frame(self.controlFrame)
        self.userInputFrame.pack(fill=X, padx=self.pad, pady=self.pad)

        self.saveFrame = Frame(self.controlFrame)
        self.saveFrame.pack(fill=X, side=BOTTOM, padx=self.pad, pady=self.pad)

        # --- Top Level Window Initialization --- #
        self.calWindow = Toplevel()
        self.calWindow.protocol("WM_DELETE_WINDOW", self.calWindow.withdraw)
        self.calWindow.attributes('-topmost', True)
        self.calWindow.withdraw()
        self.calWindow.resizable(False, False)

        self.gifWindow = Toplevel()
        self.gifWindow.protocol("WM_DELETE_WINDOW", self.gifWindow.withdraw)
        self.gifWindow.withdraw()
        self.gifWindow.attributes('-topmost', True)
        self.gifWindow.resizable(False, False)

    def initMapWidget(self):
        self.map_widget = TkinterMapView(self.mapFrame, width=850, height=850)

    def initSaveLog(self):
        self.saveText = Text(self.saveFrame, width=5, height=2)
        self.saveText.pack(fill=X)

    def placeWindowRelRoot(self, window, dx, dy):
        x = self.parent.winfo_x()
        y = self.parent.winfo_y()
        window.geometry("+%d+%d" % (x + dx, y + dy))
    
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

        Label(self.calFrame, text='DATE RANGE: ').pack(fill='both', side=LEFT)

        self.initalDateButton = Button(self.calFrame, text=f"{today.strftime('%m/%d/%Y')}", command=self.setInitialDate)
        self.initalDateButton.pack(fill='both', side=LEFT) 

        Label(self.calFrame, text='-').pack(side=LEFT)

        self.finalDateButton = Button(self.calFrame, text=f"{today.strftime('%m/%d/%Y')}", command=self.setFinalDate)
        self.finalDateButton.pack(fill='both', side=LEFT) 

        self.loadButton = Button(self.loadFrame, text="RELOAD / SAVE", command=self.onReloadClick)
        self.loadButton.pack(fill=X, side=BOTTOM) 
    
# ---------- USER CONTROL FUNCTIONALITY ---------- #
    def onReloadClick(self):
        self.saveUserInputs()
        self.setDateReport()
        self.initMapPosition()
        self.initReportButtons()

    def setDateReport(self):
        self.dateRangeReports = []
        with open(inputs['csv_file'], newline='') as csvfile:
            reports = list(reader(csvfile, delimiter=','))
            for row in reports[1:]:
                report_date = datetime.strptime(row[dateIndex], '%m/%d/%Y')
                if self.initialDate.date() <= report_date.date() <= self.finalDate.date():   # need to set initial date to start of day and final date to end of day
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
                lats.append(float(row[latIndex]))
                longs.append(float(row[longIndex]))
            lat = sum(lats) / len(lats)
            long = sum(longs) / len(longs)

        # no reports, set to lab center
        elif len(self.dateRangeReports) == 0:
            lat, long = 37.376774, -121.989967

        self.map_widget.set_position(lat, long)
        self.map_widget.set_zoom(14)
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

        gifPath = path.join(getcwd(), inputs['recording_folder'], self.attributes[i]['gifFile'])
        self.gifFrame = Label(self.gifWindow)
        self.gifFrame.pack()

        if isinstance(gifPath, str):
            img = Image.open(gifPath)

        frames = []
        self.frames = cycle(frames)
        self.delay = img.info['duration']

        try:
            for i in count(1):
                frames.append(ImageTk.PhotoImage(img.copy()))
                img.seek(i)
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

        helv10 = font.Font(family='Helvetica', size=12, weight='bold')
        Label(self.reportButtonFrame, text="DISENGAGMENT LIST", font=helv10).pack(pady=(20, 0))

        self.attributes = []
        for i, row in enumerate(self.dateRangeReports):
            lat, long = float(row[latIndex]), float(row[longIndex])

            gifFile = row[recFileIndex]
            recDateObj = datetime.strptime(gifFile.split('.')[0], '%m%d%Y_%H%M%S')
            formatRecDate = datetime.strftime(recDateObj, '%m/%d/%Y %H:%M:%S')
            title = f"{str(formatRecDate)} \n({lat}, {long})"
              
            descBox = Text(self.userInputFrame, width=5, height=5) 
            descBox.insert('1.0', row[descIndex])
            road_type = StringVar(value=row[roadIndex])
            radio1 = Radiobutton(self.userInputFrame, text='Highway', value='Highway', variable=road_type)
            radio2 = Radiobutton(self.userInputFrame, text='Street', value='Street', variable=road_type)

            marker = self.map_widget.set_marker(lat, long, marker_color_circle = 'dark green', marker_color_outside = 'green', command=partial(self.disengagmentFocus, i),)

            reportButton = Button(self.reportButtonFrame, text=title, command=partial(self.disengagmentFocus, i, None))
            reportButton.pack(fill='both', padx=self.pad, pady=self.pad) 

            attrib_dict = {
                'i': i,
                'button': reportButton,
                'marker': marker,
                'lat': lat,
                'long': long,
                'zoom': 16,
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

        Label(self.userInputFrame, text="DESCRIPTION OF REASON FOR DISENGAGMENT").pack()
        self.attributes[i]['descBox'].config(highlightthickness=3, highlightbackground = "red", highlightcolor='red')
        self.attributes[i]['descBox'].pack(fill=X)

        Label(self.userInputFrame, text="SELECT ROAD TYPE: ").pack(side=LEFT)
        self.attributes[i]['radio1'].pack(side=RIGHT)
        self.attributes[i]['radio2'].pack(side=RIGHT)

    def saveUserInputs(self):
        with open(inputs['csv_file'], newline='') as csvfile:
            reports = list(reader(csvfile, delimiter=',')) 

        for row in reports[1:]:   # skip headers
            for attrib in self.attributes:                       
                if attrib['gifFile'] == row[recFileIndex]:
                    try:
                        road_type = attrib['road_type'].get()
                        row[roadIndex] = road_type
                        
                        description = attrib['descBox'].get("1.0", END) 
                        description = description.replace('\n', '')
                        row[descIndex] = description       
                    except:
                        pass

                    break

        with open(inputs['csv_file'], 'w', newline='') as csvfile:
            csv_writer = writer(csvfile)
            csv_writer.writerows(reports)
            
        self.saveText.insert('1.0', f"SAVED: {datetime.now().strftime('%H:%M:%S') }\n") #1.0 line 1 char 0

    def onUserClose(self): 
        try:
            if messagebox.askokcancel('Disengagment GUI 2.0', "Are you sure you want to quit?\nYour work will be auto-saved.", parent=self.gifWindow):
                self.saveUserInputs()
                self.parent.destroy()
        except:
            self.parent.destroy()
            event.set() 
            exit()  # end program from logGUI

def main():
    LiveStream()

    root = Tk()
    RootWindow(root)
    root.mainloop()

main()