#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr  1 18:27:31 2020

@author: alexis
"""
from matplotlib.animation import FuncAnimation

import matplotlib.pyplot as plot
import pandas as pd

# Initialize figure
experiment = '20231218_calibracion_pid_temp2'
figure = plot.figure(constrained_layout=True)
grid = figure.add_gridspec(4, 4)

# Temperature of the heater
ax_temperatures = figure.add_subplot(grid[:2, :2])
ln_T_heater, = ax_temperatures.plot([], [], "g.")
ln_T_liquid, = ax_temperatures.plot([], [], "b.")
ln_T_sensor, = ax_temperatures.plot([], [], "r.")
ln_T_amb, = ax_temperatures.plot([], [], "c.")
ax_temperatures.set_ylabel("Temperature")

# UA
ax_biomass = figure.add_subplot(grid[:2, 2:])
ln_biomass, = ax_biomass.plot([], [],".")
ax_biomass.set_ylabel("Biomass")

# UA_loss
ax_oxygen = figure.add_subplot(grid[2:, :2])
ln_oxygen, = ax_oxygen.plot([], [], ".")
ax_oxygen.set_ylabel("Dissolved Oxygen")

# UA_loss1
ax_ph = figure.add_subplot(grid[2:, 2:])
ln_ph, = ax_ph.plot([], [], ".")
ax_ph.set_ylabel("pH")


def init():
    ax_temperatures.set_ylim(20, 70)


# Data plot
def update(i):

    data = pd.read_csv(experiment + '/data.csv')
    data = data.loc[::6].tail(2880)

    time = data.loc[:, "Time"] / 3600
    T_heater = data.loc[:, "T_heater"]
    T_liquid = data.loc[:, "T_liquid"]
    T_sensor = data.loc[:, "T_sensor"]
    T_amb = data.loc[:, "T_amb"]
    biomass = data.loc[:, "PWM_heater"]
    oxygen = data.loc[:, "Dissolved_oxygen"]
    ph = data.loc[:, "pH"]

    ln_T_heater.set_data(time, T_heater)
    ln_T_liquid.set_data(time, T_liquid)
    ln_T_sensor.set_data(time, T_sensor)
    ln_T_amb.set_data(time, T_amb)
    ln_biomass.set_data(time, biomass)
    ln_oxygen.set_data(time, oxygen)
    ln_ph.set_data(time, ph)

    ax_temperatures.relim()
    ax_temperatures.autoscale_view(scaley=False)

    ax_biomass.relim()
    ax_biomass.autoscale_view(scaley=True)

    ax_oxygen.relim()
    ax_oxygen.autoscale_view(scaley=True)

    ax_ph.relim()
    ax_ph.autoscale_view(scaley=True)


ani = FuncAnimation(figure, update, init_func=init, interval=1000 * 1)
plot.show()
