#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on 2024 Feb 29 12:00

@author: Alexis Saldivar
"""
from datetime import datetime

import time
import termios

import Board
import Handle

sqlite_db = Handle.Database(DATABASE_PATH="database.db")
board = Board.Board(address="M0", baud_rate=230400,
                    port_name="/dev/ttyUSB0")

experiment = Handle.Experiment(name="test7", sqlite_db=sqlite_db,
                               board=board)

board.start_config_observer()
time_between_samples = 1
last_time = time.time()

while True:
    try:
        current_time = time.time()
        if (current_time - last_time) >= time_between_samples:
            last_time = current_time
            data_dict = board.read_data()
            date = datetime.now()
            try:
                experiment.save_data(
                    time=date, data_dict=data_dict
                )
            except TypeError:
                print(data_dict)
        board.update_configuration()

    except KeyboardInterrupt:
        board.config_observer.stop()
        board.serial_port.close()
        sqlite_db.connection.close()

    except termios.error as e:
        print(e)
        experiment.board.reconnect()
        board.is_config_updated = True
