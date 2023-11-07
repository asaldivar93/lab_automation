#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from datetime import datetime

import pandas as pd
import arduino
import os

# Initialize Variables
print('Initializing')

# Create directory
recorder = arduino.sensors(
    ADDRESS="str", baud=230400
)
experiment_name = "test_calibration"
dir_string = experiment_name
os.mkdir(dir_string)
results_file = dir_string + '/data.csv'

# Create file to dump data
with open(results_file, 'w+') as file:
    pd.DataFrame(
        columns=['Date', 'Time', 'Vial', 'RPM', 'Stir_rate'],
    ).to_csv(file, index=False)

# Infinite loop
print('Experiment Started')
start_time = datetime.now()
while(True):
    # Every time there is a serial input
    if (recorder.sp.inWaiting() > 0):
        # Get rpm from serial string
        data = recorder.read()
        # Record Time
        date = datetime.now()  # Date
        time_delta = date - start_time  # Time since experiment started
        time = time_delta.days * 24 + time_delta.seconds / 3600  # Time in seconds

        # Set the stir rate for each vial
        stir_rate = pd.read_csv('stirrer_rate.csv', index_col="Vial")
        stir_rate = [pwm for pwm in stir_rate['pwm']]
        recorder.update_pumps(stir_rate)
        print(data[0])

        # Dump Data to file
        with open(results_file, 'a+') as file:
            for i in range(len(data)):
                pd.DataFrame(
                    [[date.strftime("%d-%m-%Y_%H-%M-%S"), time, i, data[i], stir_rate[i]]],
                    columns=['Date', 'Time', 'Vial', 'RPM', 'Stir_rate']
                ).to_csv(file, index=False, header=False)
