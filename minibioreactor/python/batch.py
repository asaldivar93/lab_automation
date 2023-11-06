#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from datetime import datetime

import pandas as pd
import arduino
import os

# Initialize Variables
print('Initializing')

# Create directory
recorder = arduino.sensors()
experiment_name = 'Biofiltro0902'
dir_string = experiment_name
os.mkdir(dir_string)
results_file = dir_string + '/data.csv'

with open(results_file, 'w+') as file:
    pd.DataFrame(
        columns=['Date', 'Time', 'PH', 'DO', 'X', 'ATM'],
    ).to_csv(file, index=False)

timer = 5  # Time period between samples in secs
last_sample = -2 * timer  # Time since last sample

pump_timer = 0.5 * 3600
pump_on = 7.5 * 60
last_pump = -2 * pump_timer
timer_on = 0
feed_on = False

print('Experiment Started')
start_time = datetime.now()
while(True):
    if (recorder.sp.inWaiting() > 0):
        timers = pd.read_csv('timers.csv', index_col="pumps")
        pump_timer = timers.loc[0, "pump_off"] * 60
        pump_on = timers.loc[0, "pump_on"] * 60
        date = datetime.now()
        time_delta = date - start_time
        time = time_delta.days * 24 + time_delta.seconds / 3600 
        vph, vdo, vx, atm = recorder.read()
        ph = 6.8966 * vph + 0.1103
        do = 50.5382 * vdo
        x = vx

        if not feed_on:
            pumps = pd.read_csv('pumps.tsv', sep='\t', index_col="pumps")
            pumps = [pwm for pwm in pumps['pwm']]
            recorder.update_pumps(pumps)

        if abs(last_pump - time * 3600) >= pump_timer:
            last_pump = time * 3600
            feed_on = True
            timer_on = time * 3600
            print("pump on")
        
        if feed_on:
           pumps[1] = 255
           recorder.update_pumps(pumps)
           if abs(timer_on - time * 3600) >= pump_on:
               print("pump off")
               pumps[1] = 0
               recorder.update_pumps(pumps)
               feed_on = False

        if abs(last_sample - time * 3600) >= timer:
            last_sample = time * 3600
            with open(results_file, 'a+') as file:
                pd.DataFrame(
                    [[date.strftime("%d-%m-%Y_%H-%M-%S"), time, ph, do, x, atm]],
                    columns=['Date', 'Time', 'PH', 'DO', 'X', 'ATM']
                ).to_csv(file, index=False, header=False)
