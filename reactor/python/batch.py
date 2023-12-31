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
    ADDRESS="r101", baud=230400
)
experiment_name = "20231123_cinetica"
dir_string = experiment_name
os.mkdir(dir_string)
results_file = dir_string + '/data.csv'

# Create file to dump data
with open(results_file, 'w+') as file:
    pd.DataFrame(
        columns=['Date', 'Time', 'Biomass', 'Dissolved_oxygen', 'pH',
                 'T_reactor', 'T_cooling_liquid', 'T_peltier', 'T_mosfet',
                 'Power', 'PWM_feed', 'PWM_heater'],
    ).to_csv(file, index=False)

pwm_values = pd.read_csv('pwm_value.csv', index_col="Channel")
last_pwm = [pwm for pwm in pwm_values['pwm']]
recorder.update_pumps(last_pwm)

temp_setpoint = pd.read_csv('temp_setpoint.csv', index_col="Channel")
last_setpoint = temp_setpoint = [temp for temp in temp_setpoint['setpoints']]
recorder.update_temp_setpoint(last_setpoint)

oxygen_bounds = pd.read_csv('oxygen_bounds.csv', index_col="Channel")
last_bounds = [bound for bound in oxygen_bounds['bounds']]
recorder.update_oxygen_bounds(last_bounds)

# If samples are sent from arduino every 0.250 seconds
# the samples per second are 4 * sample_frecuency
sample_frecuency = 20
samples = 19

# Infinite loop
print('Experiment Started')
start_time = datetime.now()
while(True):
    # Every time there is a serial input
    try:
        if (recorder.serial_port.inWaiting() > 0):
            # Get rpm from serial string
            data = recorder.read()
            samples += 1
            # Record Time
            date = datetime.now()  # Date
            time_delta = date - start_time  # Time since experiment started
            time = time_delta.days * 24 + time_delta.seconds  # Time in seconds

            # Set the PWM output
            pwm_values = pd.read_csv('pwm_value.csv', index_col="Channel")
            pwm_values = [pwm for pwm in pwm_values['pwm']]         
            recorder.update_pumps(pwm_values)

            # Set the temperature setpoint
            temp_setpoint = pd.read_csv('temp_setpoint.csv', index_col="Channel")
            temp_setpoint = [temp for temp in temp_setpoint['setpoints']]
            recorder.update_temp_setpoint(temp_setpoint)

            oxygen_bounds = pd.read_csv('oxygen_bounds.csv', index_col="Channel")
            oxygen_bounds = [bound for bound in oxygen_bounds['bounds']]
            recorder.update_oxygen_bounds(oxygen_bounds)

            # Dump Data to file
            if samples >= sample_frecuency:
                with open(results_file, 'a+') as file:
                    pd.DataFrame(
                        [[date.strftime("%d-%m-%Y_%H-%M-%S"), time,
                         data[0], data[1], data[2],
                         data[3], data[4], data[5],
                         data[6], data[7], data[8], data[9]]],
                        columns=['Date', 'Time',
                                 'Biomass', 'Dissolved_oxygen', 'pH',
                                 'T_reactor', 'T_cooling_liquid', 'T_peltier',
                                 'T_mosfet', 'Power', 'PWM_feed', 'PWM_heater'],
                    ).to_csv(file, index=False, header=False)
                samples = 0
    except OSError as e:
        print(e)
        recorder.reconnect()

        # Reset Set points
        pwm_values = pd.read_csv('pwm_value.csv', index_col="Channel")
        pwm_values = [pwm for pwm in pwm_values['pwm']]
        recorder.update_pumps(pwm_values)
        last_pwm_values = pwm_values

        
        temp_setpoint = pd.read_csv('temp_setpoint.csv', index_col="Channel")
        temp_setpoint = [temp for temp in temp_setpoint['setpoints']]
        recorder.update_temp_setpoint(temp_setpoint)
        last_setpoint = temp_setpoint

        oxygen_bounds = pd.read_csv('oxygen_bounds.csv', index_col="Channel")
        oxygen_bounds = [bound for bound in oxygen_bounds['bounds']]
        recorder.update_oxygen_bounds(oxygen_bounds)
        last_bounds = oxygen_bounds
    
          
