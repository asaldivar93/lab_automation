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
    ADDRESS="r101", baud_rate=230400, port_name="/dev/ttyUSB0"
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
