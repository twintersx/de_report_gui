import tkinter as tk
from tkcalendar import Calendar # pip install tkcalendar
from datetime import datetime, date
from tkintermapview import TkinterMapView  # pip install tkintermapview
import csv
from functools import partial

class MainApplication(tk.Frame):

    def __init__(self, parent):
        tk.Frame.__init__(self, parent)
        self.parent = parent
        self.initGUI()


    def initGUI(self):
        self.pad = 5
        self.initFrames()
        self.initCalendar()
        self.setDateReport()
        self.initMapWidget()
        self.initMapPosition()
        self.initReportButtons()
        self.initTextBox()


    def initFrames(self):   
        self.mapFrame = tk.Frame(self.parent, width=500, height=500)
        self.mapFrame.pack(side=tk.RIGHT, padx=self.pad, pady=self.pad)

        self.calWindow = tk.Toplevel()
        self.calWindow.withdraw()

        self.controlFrame = tk.Frame(self.parent)
        self.controlFrame.pack(fill='both', side=tk.LEFT, padx=self.pad, pady=self.pad)

        self.calFrame = tk.Frame(self.controlFrame, height=20)
        self.calFrame.pack(fill=tk.X, side=tk.TOP, padx=self.pad, pady=self.pad)

        self.loadFrame = tk.Frame(self.controlFrame)
        self.loadFrame.pack(fill=tk.X, side=tk.TOP, padx=self.pad, pady=self.pad)

        self.reportButtonFrame = tk.Frame(self.controlFrame)
        self.reportButtonFrame.pack(fill='both', padx=self.pad, pady=self.pad)

        self.userInputFrame = tk.Frame(self.controlFrame)
        self.userInputFrame.pack(fill='both', side=tk.BOTTOM, padx=self.pad, pady=self.pad)
    

    def closeInitialCal(self, event):
        self.initialDate = datetime.strptime(self.cal.get_date(), '%m/%d/%y')
        self.initalDateButton.config(text=self.initialDate.strftime('%m/%d/%y'))
        self.calWindow.withdraw()


    def closeFinalCal(self, event):
        self.finalDate = datetime.strptime(self.cal.get_date(), '%m/%d/%y')
        self.finalDateButton.config(text=self.finalDate.strftime('%m/%d/%y'))
        self.calWindow.withdraw()


    def setInitialDate(self):
        self.calWindow.deiconify()
        self.calWindow.bind('<Button-1>', self.closeInitialCal)


    def setFinalDate(self):
        self.calWindow.deiconify()
        self.calWindow.bind('<Button-1>', self.closeFinalCal)


    def initCalendar(self):
        today = date.today()
        self.initialDate, self.finalDate = datetime.now(), datetime.now()
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

    
    def onDateChangeClick(self):
        self.setDateReport()
        self.initMapPosition()
        self.initReportButtons()


    def setDateReport(self):
        self.report = []
        with open('de_reports.csv', newline='') as csvfile:
            reader = list(csv.reader(csvfile, delimiter=','))
            self.header = reader[0]
            date_index = self.header.index("DATE")
            for row in reader[1:]:
                report_date = datetime.strptime(row[date_index], '%d/%m/%Y')
                if self.initialDate <= report_date <= self.finalDate:
                    self.report.append(row)

    def initMapWidget(self):
        self.map_widget = TkinterMapView(self.mapFrame, width=500, height=500)
        self.lat_index = self.header.index("LATITUDE")
        self.long_index = self.header.index("LONGITUDE")

    def changeMapPosition(self, lat, long, zoom):
        self.map_widget.set_position(lat, long)
        self.map_widget.set_zoom(zoom)
        self.map_widget.pack()
    
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

        self.changeMapPosition(lat, long, 13)
    
    def addMapMarkers(self, i):
        marker = self.markers[i]
        marker.setMarker(marker_color_circle='green', marker_color_outside='green')
        self.markers[i] = marker

    def initReportButtons(self):
        self.clearWidgets(self.reportButtonFrame)
        self.markers = []        
        for i, row in enumerate(self.report):
            lat, long = float(row[self.lat_index]), float(row[self.long_index])
            text = f"{lat}, {long}" 
            self.markers.append(self.map_widget.set_marker(lat, long))
            reportButton = tk.Button(self.reportButtonFrame, text=text, command=partial(self.changeMapPosition, lat, long, 15))
            reportButton.pack(fill='both', padx=self.pad, pady=self.pad) 

    def clearWidgets(self, frame):
        self.map_widget.delete_all_marker()

        widgets = frame.winfo_children()
        for w in widgets:
            w.destroy()

    def initTextBox(self):
        tk.Label(self.userInputFrame, text="DESCRIPTION OF REASON FOR DISENGAGMENT").pack(fill='both')
        tk.Entry(self.userInputFrame).pack(fill='both')
            
if __name__ == "__main__":
    root = tk.Tk()
    MainApplication(root).pack(side='top', fill='both', expand=True)
    root.mainloop()