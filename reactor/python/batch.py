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

board_0 = Board.Board(
    "M0", baud_rate=230400, port_name="/dev/ttyUSB0", config_dir="config_M0/"
    )
board_1 = Board.Board(
    "M1", baud_rate=230400, port_name="/dev/ttyUSB1", config_dir="config_M1/"
    )
boards = [board_0, board_1]
experiment = Handle.Experiment(name="Calibracion_OD", sqlite_db=sqlite_db,
                               boards=boards)
for board in boards:
    board.start_config_observer()

time_between_samples = 0.5
last_time = time.time()

while True:
    try:
        current_time = time.time()
        if (current_time - last_time) >= time_between_samples:
            last_time = current_time

            data_dict = {}
            for board in boards:
                try:
                    data_dict = data_dict | board.read_data()
                except termios.error as e:
                    Board.logging.warning(e)
                    board.reconnect()
                    board.is_config_updated = True

            date = datetime.now()

            try:
                experiment.save_data(
                    time=date, data_dict=data_dict
                )
            except TypeError:
                print(data_dict)
        for board in boards:
            board.update_configuration()

    except KeyboardInterrupt:
        board.config_observer.stop()
        board.serial_port.close()
        sqlite_db.connection.close()
