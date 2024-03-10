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
sample_period = experiment.get_sample_period(period_time_seconds=1)
sample_counter = 0
while True:
    try:
        if (experiment.board.serial_port.inWaiting() > 0):
            time = datetime.now()
            data_dict = experiment.board.read_data()
            if data_dict and sample_counter >= sample_period:
                experiment.save_data(
                    time=time, data_dict=data_dict
                )
                sample_counter = 0
            experiment.board.update_configuration()
            sample_counter += 1
    except KeyboardInterrupt:
        board.config_observer.stop()
        board.serial_port.close()
        sqlite_db.connection.close()

    except OSError as e:
        print(e)
        experiment.board.reconnect()
