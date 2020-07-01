# -*- coding: utf-8 -*-
"""
Created on Sun Jun 21 22:27:47 2020

@author: migue
"""

#         ┌─────── PID1 ──────┐┌─────── PID2 ──────┐┌── Motores ───┐ Máx/Mín Vel. Fijas  PID S/N┌─── Giroscopio ────┐
#     P Ac Kp1    Ki1    Kd1    Kp2    Ki2    Kd2    M1  M2  M3  M4  VM  Vm  V1  V2  V3  1 2 3 4 EjeX   EjeY   EjeZ   Chk
#Tx: $0/ON/000000/000000/000000/000000/000000/000000/000/000/000/000/500/000/102/151/204/0/0/0/0/0.7055/0.5334/0.5795/5486#

import threading, time

import serial as ser
import serial.tools.list_ports

import tkinter as tk
import tkinter.font as tkFont

from enum import Enum

import numpy as np

import matplotlib.pyplot as plt
import matplotlib.animation as animation


#COMMANDS DEFINITION
    
class Commands():
    
    allowedCommands = {"debug", "availablePorts", "connect", "disconnect", "startCom", "stopCom", "status", "ping", "raw", "setMode", "quickStartUp", "setValues", "setPIDPwr", "graphs", "setKs"}
    
    def debug(*_argList):
        argList = _argList[0]
        print(str(argList[0]).strip('[\']'))
        print(type(argList))
        print(argList[0])
    
    def availablePorts(*_argList):
        argList = _argList[0]
        comPorts = list(serial.tools.list_ports.comports())
        
        for port in comPorts:
            prompt("%s : %s" % (port.device, port.description))
        
    def connect(*_argList):
        argList = _argList[0]
        port = str(argList[0]).strip('\'[]')
        print(port)
        
        s.port = port
        s.baudrate = 9600
        s.timeout = 2
        s.open()
        
    def disconnect(*_argList):
        argList = _argList[0]
        s.close()
    
    def quickStartUp(*_argList):
        pass
        
    
    def startCom(*_argList): 
        argList = _argList[0]
        r.start()
        
    def stopCom(*_argList):
        argList = _argList[0]
        pass
        #r.join()
    
    def reader():
        while True:
                    
            if s.in_waiting > 0:
                msg = s.read_until()
                msg = msg.decode("ascii") 
                if "gr" in msg:
                    #prompt(msg)
                    Commands.grapher(msg)
                    
                else: 
                    prompt(msg)
                #TextWidget.see(tk.END)
            else:
                time.sleep(0.01)
                 
    def status(*_argList):
        argList = _argList[0]
        Commands.send("2")
    
    def ping(*_argList):
        argList = _argList[0]
        Commands.send("1")
        
#    def raw(*_argList):
#        argList = _argList[0]
#        msg = ""
#        for i in range(len(argList)):
#            msg = msg + str(argList[i]).strip('\'[], ')
#                           
#            if not i == len(argList)-1:
#                msg = msg + "/"
#                                
#        Commands.send(msg)
        
#    def setValues(*_argList):
#        argList = _argList[0]
#        msg = ""
#        for i in range(len(argList)):
#            msg = msg + str(argList[i]).strip('\'[], ')
#                           
#            if not i == len(argList)-1:
#                msg = msg + "/"
#            
#        Commands.makeStandardMsg()
        
    def setKs(*_argList):
        argList = _argList[0]
        global kp, ki, kd
        
        kp = float(argList[0])
        ki = float(argList[1])
        kd = float(argList[2])
        
        Commands.makeStandardMsg()
        
    def setPIDPwr(*_argList):
        argList = _argList[0]
        
        global PID1
        
        if str(argList[0]).lower() == "on":
            PID1 = 1
            
        elif str(argList[0]).lower() == "off":
            PID1 = 0  
        
        Commands.makeStandardMsg()
        
        
    def setMode(*_argList):
        argList = _argList[0]
        global mode
        
        mode = argList[0]
        
        Commands.makeStandardMsg()
        
    def makeStandardMsg():
        msg = "0/" + mode + "/" + str(kp) + "/" + str(ki) + "/" + str(kd) + "/0/0/0/" + str(thrust) + "/" + \
        str(thrust) + "/" + str(thrust) + "/" + str(thrust) + "/" + str(vMax) + "/" + str(vMin) + "/" + \
        "0/0/0/" + str(PID1) + "/" + "0/0/0"
        
        Commands.send(msg)
        
    def send(msg):
        chkSum = Commands.getChecksum(msg)
        msg = "$" + msg + "/" + str(chkSum) + "#"
        
        print(msg)
        
        bMsg = bytes(msg, 'ascii')
        s.write(bMsg)
        
        
    def getChecksum(msg):
        bMsg = bytes(msg, 'ascii')
        chkSum = 0
        for byte in bMsg:
            chkSum = chkSum + byte
            
        print("CheckSum:" + str(chkSum))
        return chkSum

#    def graphs(*_argList):
#        global data, graphing, graphingIndex, fig
#        
#        data = np.zeros([5, 1000])
#        graphing = True
#        graphingIndex = 0
#        
#        Commands.send("15")
#            
#    def grapher(msg):
#        global data, graphingIndex
#        msg = msg.strip('$#')
#        _data = msg.split('/')
#        
#        if graphingIndex < 999:  
#            for i in range(data.shape[0]):
#                data[i][graphingIndex] = float(_data[i+1])
#        
#            data[0][graphingIndex] = data[0][graphingIndex]/1000000
#            
#            graphingIndex = graphingIndex + 1
#            
#        elif graphingIndex == 999:
#            aux = np.delete(data, 0, 1)
#            data = [aux, _data[1:]]
            
        
#GLOBAL VARIABLES

s = ser.Serial()
r = threading.Thread(target=Commands.reader, name = "Reader")
comStarted = False

#data = np.zeros([5, 1000])
#
#graphing = False
#graphingIndex = 0
#
#fig, (ax1, ax2) = plt.subplots(2, 1)
#fig.subplots_adjust(hspace=0.5)


#def animate(i):
#        
#        if graphing:
#    
#            x = data[0]
#            y1 = data[1]
#            y2 = data[2]
#            
#            ax1.clear()
#            ax1.plot(x, y1)
#            ax1.plot(x, y2)
#            
#            y3 = data[3]
#            y4 = data[4]
#            
#            ax2.clear()
#            ax2.plot(x, y3)
#            ax2.plot(x, y4)   
#
#ani = animation.FuncAnimation(fig, animate, interval = 5)
#plt.show()

#DEFAULT VALUES

class variableNumbers(Enum):
    mode = 1
    kp = 2
    ki = 3
    kd = 4
    thrust = 8
    vMax = 12
    vMin = 13
    PID1 = 17

variables = []

mode = "St"

kp = 0
ki = 0
kd = 0

thrust = 0

vMax = 1250
vMin = 500

PID1 = 0  

#APP CLASS FOR TKINTER

class App(tk.Frame):
    
    def __init__(self, master = None):
        super().__init__(master)
        self.master = master
        self.pack()
        self.putWidgets()
        
    def putWidgets(self):
        self.boldFont = tkFont.Font(family="Courier", size = 10, weight = "bold")
        
        self.incoming = tk.Text(self, state = "disabled")
        self.outcoming = tk.Entry(self)
        self.scrollbar = tk.Scrollbar(self)
        
        self.incoming.grid(row = 0, column = 0, sticky = "nesw")
        self.outcoming.grid(row = 1, column = 0, sticky = "nesw")
        self.scrollbar.grid(row = 0, column = 1, sticky = "ns")
        
        self.grid_columnconfigure(0, weight = 1)
        self.grid_columnconfigure(1, weight = 1)
        self.grid_rowconfigure(0, weight = 1)
        self.grid_rowconfigure(1, weight = 1)
        
        self.incoming.config(yscrollcommand = self.scrollbar.set)
        self.scrollbar.config(command=self.incoming.yview)
        
        self.incoming.config(state="normal")
        self.incoming.insert(tk.END, "Ground Control for ArduDron by Miguel Sánchez\n\n")
        self.incoming.tag_add("BOLD", "1.0", tk.INSERT)
        self.incoming.tag_configure("BOLD", font=self.boldFont)
        self.incoming.config(state = "disabled")

root = tk.Tk()
root.title("Ground Control")

#GUI METHOD DEFINITIONS

def keydown(e):
    if e.keycode == 13:
        txt = app.outcoming.get()
        app.outcoming.delete(0, tk.END)
        
        app.incoming.config(state="normal")
        app.incoming.insert(tk.END, "\n--> ")
        startIndx = str(app.incoming.index(tk.INSERT)) + "linestart"
        app.incoming.tag_add("BOLD", startIndx, tk.INSERT)
        #app.incoming.tag_delete("BOLD", tk.INSERT, tk.END)
        app.incoming.insert(tk.END, txt)
        app.incoming.config(state="disabled")
        
        app.incoming.see(tk.END)
        
        processCommand(txt)

def prompt(_txt):
    print(_txt)
    
    app.incoming.config(state="normal")
    app.incoming.insert(tk.END, "\n" + _txt)
    app.incoming.config(state="disabled")
    app.incoming.see(tk.END)

def processCommand(_txt):
    txt = _txt.strip()
    command = txt.split()
    
    if command[0] in Commands.__dict__ and command[0] in Commands.allowedCommands:
        verb = getattr(Commands, command[0])
        argList = []
        for i in range(len(command) - 1):
            #print(command[i+1])
            argList.append(command[i+1])
        verb(argList)
        
    else:
        prompt(command[0] + " command does not exist")
       
#APP CONFIGURATION AND MAINLOOP START     

app = App(master = root)

app.outcoming.focus_set()

app.outcoming.bind("<Key>", keydown)
app.incoming.bind("<Key>", keydown)

app.mainloop()
