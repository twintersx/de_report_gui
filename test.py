import tkinter as tk
from PIL import Image, ImageTk
from itertools import count, cycle
from time import sleep
 
class ImageLabel(tk.Label):


    def load(self, im):

        if isinstance(im, str):
            im = Image.open(im)
        frames = []

        try:
            for i in count(1):
                frames.append(ImageTk.PhotoImage(im.copy()))
                im.seek(i)
        except EOFError:
            pass
        self.frames = cycle(frames)
 
        try:
            self.delay = im.info['duration']
        except:
            self.delay = 100
 
        if len(frames) == 1:
            self.config(image=next(self.frames))
        else:
            self.next_frame()
 
    def unload(self):
        self.config(image=None)
        self.frames = None
 
    def next_frame(self):
        if self.frames:
            self.config(image=next(self.frames))
            self.after(self.delay, self.next_frame)
 
#demo :
root = tk.Tk()
lbl = ImageLabel(root) #put imagelabel into marker? 
lbl.pack()
#tk.Button(lbl, command=sleep(2), height=10, width=10).pack()
lbl.load(r'C:\Users\X076979\OneDrive - Nissan Motor Corporation\Desktop\Projects\de_report_gui\recordings\01272023_154237.gif')
root.mainloop()

#instead of putting in label as our vessel to display image, 
# our vessel needs to be the marker