import tkinter as tk
from time import sleep

window = tk.Tk()

def open_command():
    open_btn.config(bg='green')
    sleep(3)
    close_btn.config(bg='white')

def close_command():
    open_btn.config(bg='white')
    close_btn.config(bg='red')

font=('Times New Roman', 12)
open_btn = tk.Button(window, text='Open', font=font, fg='green', bg='white', width=5, command=open_command)
open_btn.pack()
close_btn = tk.Button(window, text='Close', font=font, fg='red', bg='white', width=5, command=close_command)
close_btn.pack()

window.mainloop()