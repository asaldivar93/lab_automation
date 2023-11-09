#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Oct  7 20:19:46 2019

@author: Alexis
"""

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
        self.sp = serial.Serial(port=port, baudrate=baud, timeout=2)
        self.sp.flushInput()
        self.sp.flushOutput()
        time.sleep(2)

    def get(self):
        cmd_str = self.build_cmd_str('2', '')
        try:
            self.sp.write(cmd_str.encode())
            self.sp.flushInput()
        except Exception:
            return None
        return self.sp.readline().decode('UTF-8')

    def readline_(self):
        return self.sp.readline().decode('UTF-8')

    def read(self):
        data = self.sp.readline().decode('UTF-8')
        self.sp.flushInput()
        data = data.split(" ")[1].split(",")[1:9]

        return np.asarray(data, dtype=np.float64, order='C')

    def update_pumps(self, pwm):
        self.write('1', pwm)
        return pwm

    def write(self, cmd, pwm):
        cmd_str = self.build_cmd_str(cmd, pwm)
        try:
            self.sp.write(cmd_str.encode())
            self.sp.flush()
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
