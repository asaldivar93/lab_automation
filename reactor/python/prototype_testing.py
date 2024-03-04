#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on 2024 Feb 29 12:00

@author: Alexis Saldivar
"""
from datetime import datetime

import pandas as pd

import Board
import Handle


sqlite_db = Handle.Database(
    DATABASE_PATH="database.db"
)

board = Board.Board(
    address="r101", baud_rate=230400, port_name="/dev/ttyUSB0"
)

experiment = Handle.Experiment(
    name="test", sqlite_db=sqlite_db, board=board
)


while True:
    if (experiment.board.serial_port.inWaiting() > 0):
        time = datetime.now()
        data_dict = experiment.board.read_data()
        if data_dict:
            experiment.save_data(
                time=time, data_dict=data_dict
            )

board.serial_port.close()

a = sqlite_db.cursor.execute("SELECT * FROM test").fetchall()
pd.DataFrame(a)

board.request_board_info()
valid_control_modes = {"MANUAL": 0, "TIMER": 1, "PID": 2, "ONOFF": 3}
list(valid_control_modes.keys())[0]
a = {"address": "r101"
     "outputs": [("pwm",0,13,0), ("pwm",1,12,0),
                 ("pwm",2,14,0), ("pwm",3,27,2),
                 ("pwm",4,26,0), ("pwm",5,15,0), ],
     "inputs": [("analog",0"current"),("analog",1"dissolved_oxygen"),("analog",2"ph"),("analog",3"temperature_0"),("analog",4"temperature_1"),("analog",5"temperature_2"),("analog",6"temperature_3"),("analog",7"temperature_4"),]}
