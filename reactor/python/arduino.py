#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Oct  7 20:19:46 2019

@author: Alexis
"""

from serial.tools import list_ports

import time
import numpy as np
import serial



class sensors(object):
    def __init__(
        self,
        ADDRESS,
        port='/dev/ttyUSB0',
        baud=230400,
    ):
        print('Opening connection')
        self.ADDRESS = ADDRESS
        self.port = port
        self.baud = baud
        self.serial_port = serial.Serial(
            port=self.port, baudrate=self.baud, timeout=2
        )
        self.serial_port.flushInput()
        self.serial_port.flushOutput()
        time.sleep(0.5)
    
    def reconnect(self):
        
        time.sleep(1)
        self.serial_port.close()
        time.sleep(2)
        print("Serial Port Closed")
        self.serial_port = None
        
        while self.serial_port == None:
            time.sleep(5)
            print('Opening connection')
            try:
                ports = list_ports.comports()
                port = ports[0].device
                self.serial_port = serial.Serial(
                    port=self.port, baudrate=self.baud, timeout=2
                )
            except IndexError:
                print("Line connection Lost")
        self.serial_port.flushInput()
        self.serial_port.flushOutput()
        print("Device Connected")
        time.sleep(0.5)
    
    def get(self):
        cmd_str = self.build_cmd_str('2', '')
        try:
            self.serial_port.write(cmd_str.encode())
            self.serial_port.flushInput()
        except Exception:
            return None
        return self.serial_port.readline().decode('UTF-8')

    def readline_(self):
        return self.serial_port.readline().decode('UTF-8')

    def read(self):
        data = self.serial_port.readline().decode('UTF-8')
        self.serial_port.flushInput()
        data = data.split(" ")[1].split(",")[1:-2]

        return np.asarray(data, dtype=np.float64, order='C')

    def update_pumps(self, pwm):
        self.write('1', pwm)
        return pwm

    def update_temp_setpoint(self, setpoint):
        self.write('2', setpoint)
        return setpoint

    def update_oxygen_bounds(self, bounds):
        self.write('3', bounds)
        return bounds

    def write(self, cmd, pwm):
        cmd_str = self.build_cmd_str(cmd, pwm)
        try:
            self.serial_port.write(cmd_str.encode())
            self.serial_port.flush()
        except:
            return None
        return None

    def build_cmd_str(self, cmd, args=None):
        """
        Build a command string that can be sent to the arduino.

        Input:
            cmd (str): the command to send to the arduino, must not
                contain a % character
            args (iterable): the arguments to send to the command
        """
        if args:
            args = ','.join(map(str, args))
        else:
            args = ''
        return self.ADDRESS + " {cmd},{args}!\n".format(cmd=cmd, args=args)
