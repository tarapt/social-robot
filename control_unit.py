from Tkinter import *
from threading import Thread

class ControlUnit:
    def __init__(self):
        self.setup_gui()        
        print("Hello")
    
    def setup_gui(self):
        self.gui = Tk()
        theLabel = Label(self.gui, text="This is too easy")
        theLabel.pack()
        Thread(target=self.start_gui).start()

    def start_gui(self):
        self.gui.mainloop()

    def stop(self):
        self.gui.destroy()

if __name__ == '__main__':
    control_unit = ControlUnit()