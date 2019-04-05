import tkinter as tk
from tkinter.ttk import Radiobutton
from tkinter import messagebox, Scale
import atexit

from source3 import *

"""class Robot:
    def forward(self):
        pass

    def turnLeft(self):
        pass

    def turnRight(self):
        pass

    def backward(self):
        pass

    def stop(self):
        pass

    def shutdown(self):
        pass

    def flyWheelsOn(self):
        pass

    def flyWheelsOff(self):
        pass"""


class NoHoverButton(tk.Button):
    def __init__(self, master, **kw):
        tk.Button.__init__(self,master=master,**kw)
        self.defaultBackground = self["background"]
        self['activebackground'] = self["background"]

    def setBackground(self, colour):
        self["activebackground"] = colour
        self["background"] = colour

    def resetBackground(self):
        self.setBackground(self.defaultBackground)


class App:
    def __init__(self):
        self.root = tk.Tk()

        self.robot = Robot()
        atexit.register(self.robot.shutdown)

        self.TITLE = "Robot control"
        self.root.title = self.TITLE
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self._frame = self.getNewFrame()
        self.main()

    def getFrame(self):
        return self._frame

    def getNewFrame(self):
        #Create a Frame
        frame = tk.Frame(self.root, width=600, height=600)
        frame.pack(fill="both", expand=True)
        frame.grid_propagate(False)
        return frame

    def main(self):
        self.flywheelsFlag = 0
        self.flywheelsDuty = 1130

        def toggleFlywheels(event=None):
            self.flywheelsFlag = (self.flywheelsFlag+1)%2
            self.flywheelsDuty = self.slider.get()
            if self.flywheelsFlag:
                self.flwheels.setBackground("red")
            else:
                self.flwheels.resetBackground()
            print("On" if self.flywheelsFlag else "Off", self.flywheelsDuty)

            if self.flywheelsFlag:
                #Change source2 api to include variable flywheel duty
                self.robot.flyWheelsOn(self.flywheelsDuty)
            else:
                self.robot.flyWheelsOff()

        def clicked(event=None):
            pass

        def unclicked(event=None):
            self.robot.stop()

        self.up = NoHoverButton(self._frame, text="^", height=10, width=10)
        self.up.grid(column=1, row=1)
        self.up.bind("<ButtonPress>", lambda x: self.robot.forward())
        self.up.bind("<ButtonRelease>", unclicked)

        self.left = NoHoverButton(self._frame, text="<", height=10, width=10)
        self.left.grid(column=0, row=2)
        self.left.bind("<ButtonPress>", lambda x: self.robot.turnLeft())
        self.left.bind("<ButtonRelease>", unclicked)

        self.right = NoHoverButton(self._frame, text=">", height=10, width=10)
        self.right.grid(column=2, row=2)
        self.right.bind("<ButtonPress>", lambda x: self.robot.turnRight())
        self.right.bind("<ButtonRelease>", unclicked)

        self.down = NoHoverButton(self._frame, text="v", height=10, width=10)
        self.down.grid(column=1, row=3)
        self.down.bind("<ButtonPress>", lambda x: self.robot.backward())
        self.down.bind("<ButtonRelease>", unclicked)

        self.flwheels = NoHoverButton(self._frame, text="*", height=10, width=10)
        self.flwheels.grid(column=1, row=2)
        self.flwheels.bind("<ButtonPress>", toggleFlywheels)

        self.slider = Scale(self._frame, from_=1000, to=2000, tickinterval=200)
        self.slider.set(self.flywheelsDuty)
        self.slider.grid(column=3, row=2)

    def close(self):
        self.robot.stop()
        self.robot.flyWheelsOff()
        messagebox.showerror('Closing window!', 'You are closing the window')
        exit()

def main():
    app = App()
    app.root.mainloop()

if __name__ == "__main__":
    exit(main())
