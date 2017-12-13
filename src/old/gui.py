#! /usr/bin/env python
from glob import glob
import time
import sys
import math
from tkinter import *
from tkinter import ttk
import _thread
from subprocess import call

class LineFollowerGUI():
    
    def __init__(self):
        self.status = "Started"
        self.battery = "N/A"
        self.facingTowardsA = False;
        self.last_state_published = "Stop";
        self.current_state = "Stop";

        self.root = Tk()
        self.root.title("GUI")
        
        framew = 500; # root w
        frameh = 400; # root h
        screenw = self.root.winfo_screenwidth();
        screenh = self.root.winfo_screenheight();
        posx = (screenw/2) - (framew/2);
        posy = (screenh/2) - (frameh/2);
        self.root.geometry( "%dx%d+%d+%d" % (framew,frameh,posx,posy))

        self.currentStatus = StringVar();
        self.currentStatus.set('Stop');

        self.batteryStatus = StringVar();
        self.batteryStatus.set('na%');
        
        self.mainframe = ttk.Frame(self.root, padding="10 10 30 30", height=400, width=500)
        self.mainframe.grid(column=0, row=0, sticky=(N, W, E, S))
        self.mainframe.columnconfigure(0, weight=50,minsize=50)
        self.mainframe.rowconfigure(0, weight=50,minsize=50)

        self.buttonStyle = ttk.Style()
        self.buttonStyle.configure('my.TButton', font=('Helvetica', 18))
        
        self.batteryLabel = ttk.Label(self.mainframe, textvariable=self.batteryStatus, font=('Helvetica',12));
        self.batteryLabel.grid(row=1, column=1);

        self.statusLabel = ttk.Label(self.mainframe, textvariable=self.currentStatus, font=('Helvetica',12));
        self.statusLabel.grid(row=1, column=0);

        ttk.Button(self.mainframe, text="Go To A", style='my.TButton', command=self.goToA, width=16).grid(row=2, rowspan=2, column=0, pady=25)
        ttk.Button(self.mainframe, text="Go To B/DOCK", style='my.TButton', command=self.goToB, width=16).grid(row=2, rowspan=2, column=1, pady=25)
        ttk.Button(self.mainframe, text="STOP", style='my.TButton', command=self.Stop, width=16).grid(row=4, rowspan=3, column=0, columnspan=3, pady=5)
        
        self.root.after(1000, self.updateLabel);
    
    def goToA(self):
        print("if not facing towards A ( turn 180), start following line, publish, turn, or drive state");
        self.batteryStatus.set("na%");
        self.current_state = "GoToA";

    def goToB(self):
        print("if facing towards A (turn 180), start following Line, publish, turn or follow line state");
        self.current_state = "GoToB";

    def Stop(self):
        print("stop");
        self.current_state = "Stop";

    def updateLabel(self):
        if(self.current_state != self.last_state_published):
            call(["rostopic", "pub", "-1", "/gui_state", "std_msgs/String", self.current_state])
            self.last_state_published = self.current_state;

        self.currentStatus.set(self.current_state);
        self.statusLabel.update_idletasks();
        self.batteryLabel.update_idletasks();
    
        self.root.update_idletasks();
        self.root.after(200, self.updateLabel);

    def __del__(self):
        pass;

gui = LineFollowerGUI();
gui.root.mainloop();

