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
        port=None,
        baud=57600,
    ):
        print('Opening connection')
        self.sp = serial.Serial(port='/dev/ttyUSB0', baudrate=baud, timeout=2)
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
        data = data.split("\t")
        data = np.asarray(data, dtype=np.float64, order='C')
        vph = data[1]
        vdo = data[0]
        vx = data[2]
        atm = data[3]
        return [vph, vdo, vx, atm]

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
        return "appl {cmd},{args}!\n".format(cmd=cmd, args=args)

    def close(self):
        try:
            self.sp.close()
            print('Arduino disconnected successfully')
        except:
            print('Problems disconnecting from Arduino.')
            print('Please unplug and reconnect Arduino.')
        return True
