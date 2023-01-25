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

    def initFrames(self):        
        self.controlFrame = tk.Frame(self.parent, width=120, highlightbackground="grey", highlightthickness=2)
        self.controlFrame.pack(fill=tk.Y, side=tk.LEFT, padx=self.pad, pady=self.pad)
        self.mapFrame = tk.Frame(self.parent, width=500, height=500)
        self.mapFrame.pack(side=tk.RIGHT, padx=self.pad, pady=self.pad)
    
    def initWidgets(self):
        today = date.today()
        self.cal = Calendar(self.controlFrame, selectmode='day', year=today.year, month=today.month, day=today.day)
        self.cal.pack(side=tk.TOP, padx=self.pad, pady=self.pad)

        dateButton = tk.Button(self.controlFrame, text="Change Date", command=self.setDateReport)
        dateButton.pack(fill=tk.X, side=tk.TOP, padx=self.pad, pady=self.pad)

    def setDateReport(self):
        map_date = datetime.strptime(self.cal.get_date(), '%m/%d/%y')

        report = []
        with open('de_reports.csv', newline='') as csvfile:
            reader = list(csv.reader(csvfile, delimiter=','))
            date_index = reader[0].index("DATE")
            for row in reader[1:]:
                report_date = datetime.strptime(row[date_index], '%d/%m/%Y')
                if report_date >= map_date:
                    report.append(row)

        print(report)


    def initMap(self):
        # Map
        map_widget = TkinterMapView(self.mapFrame, width=500, height=500)
        map_widget.pack()

if __name__ == "__main__":
    root = tk.Tk()
    MainApplication(root).pack(side='top', fill='both', expand=True)
    root.mainloop()
