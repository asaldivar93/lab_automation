#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on 2024 Feb 29 12:00

@author: Alexis Saldivar
"""
from datetime import datetime

import Board
import Handle

sqlite_db = Handle.Database(DATABASE_PATH="database.db")
board = Board.Board(address="M0", baud_rate=230400,
                    port_name="/dev/ttyUSB0")

experiment = Handle.Experiment(name="test", sqlite_db=sqlite_db,
                               board=board)

experiment.board.start_config_observer()
while True:
    try:
        if (experiment.board.serial_port.inWaiting() > 0):
            time = datetime.now()
            data_dict = experiment.board.read_data()
            if data_dict:
                experiment.save_data(
                    time=time, data_dict=data_dict
                )
            experiment.board.update_configuration()
    except KeyboardInterrupt:
        board.config_observer.stop()
        board.serial_port.close()
