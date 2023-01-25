import tkinter as tk
from tkcalendar import Calendar # pip install tkcalendar
from datetime import datetime, date
from tkintermapview import TkinterMapView  # pip install tkintermapview
import csv

class MainApplication(tk.Frame):
    def __init__(self, parent):
        tk.Frame.__init__(self, parent)
        self.parent = parent
        self.pad = 5
        self.initFrames()
        self.initWidgets()
        self.setDateReport()
        self.setMap()

    def initFrames(self):        
        self.controlFrame = tk.Frame(self.parent, width=120, highlightbackground="grey", highlightthickness=2)
        self.controlFrame.pack(fill=tk.Y, side=tk.LEFT, padx=self.pad, pady=self.pad)
        self.mapFrame = tk.Frame(self.parent, width=500, height=500)
        self.mapFrame.pack(side=tk.RIGHT, padx=self.pad, pady=self.pad)
    
    def initWidgets(self):
        today = date.today()
        self.cal = Calendar(self.controlFrame, selectmode='day', year=today.year, month=today.month, day=today.day)
        self.cal.pack(side=tk.TOP, padx=self.pad, pady=self.pad)

        dateButton = tk.Button(self.controlFrame, text="Change Date", command=self.onDateChangeClick)
        dateButton.pack(fill=tk.X, side=tk.TOP, padx=self.pad, pady=self.pad)

    def onDateChangeClick(self):
        self.setDateReport()
        self.changeMapPosition()

    def setDateReport(self):
        map_date = datetime.strptime(self.cal.get_date(), '%m/%d/%y')

        self.report = []
        with open('de_reports.csv', newline='') as csvfile:
            reader = list(csv.reader(csvfile, delimiter=','))
            self.header = reader[0]
            date_index = self.header.index("DATE")
            for row in reader[1:]:
                report_date = datetime.strptime(row[date_index], '%d/%m/%Y')
                if report_date >= map_date:
                    self.report.append(row)

    def changeMapPosition(self):
        lat_index = self.header.index("LATITUDE")
        long_index = self.header.index("LONGITUDE")

        # at least one report
        if len(self.report) >= 1:
            lats, longs = [], []
            for row in self.report:
                lats.append(float(row[lat_index]))
                longs.append(float(row[long_index]))
            latPos = sum(lats) / len(lats)
            longPos = sum(longs) / len(longs)

        # no reports, set to lab center
        elif len(self.report) == 0:
            latPos, longPos = 37.376774, -121.989967

        self.map_widget.set_position(latPos, longPos)
        self.map_widget.set_zoom(14)
            
    def setMap(self):
        self.map_widget = TkinterMapView(self.mapFrame, width=500, height=500)
        self.changeMapPosition()
        self.map_widget.pack()

if __name__ == "__main__":
    root = tk.Tk()
    MainApplication(root).pack(side='top', fill='both', expand=True)
    root.mainloop()