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
experiment = 'test'
figure = plot.figure(constrained_layout=True)
grid = figure.add_gridspec(4, 4)

# Temperature of the heater
ax_temperatures = figure.add_subplot(grid[:2, :2])
ln_T_liquid, = ax_temperatures.plot([], [], "g")
ln_T_cooler, = ax_temperatures.plot([], [], "b")
ax_temperatures.set_ylabel("Temperature")

# UA
ax_biomass = figure.add_subplot(grid[:2, 2:])
ln_biomass, = ax_biomass.plot([], [])
ax_biomass.set_ylabel("Biomass")

# UA_loss
ax_oxygen = figure.add_subplot(grid[2:, :2])
ln_oxygen, = ax_oxygen.plot([], [])
ax_oxygen.set_ylabel("Dissolved Oxygen")

# UA_loss1
ax_ph = figure.add_subplot(grid[2:, 2:])
ln_ph, = ax_ph.plot([], [])
ax_ph.set_ylabel("pH")


def init():
    ax_temperatures.set_ylim(0, 55)


# Data plot
def update(i):

    data = pd.read_csv(experiment + '/data.csv')
    data = data.loc[::6].tail(2880)

    time = data.loc[:, "Time"] / 3600
    T_liquid = data.loc[:, "T_reactor"]
    T_cooler = data.loc[:, "T_cooling_liquid"]
    biomass = data.loc[:, "Biomass"]
    oxygen = data.loc[:, "Dissolved_oxygen"]
    ph = data.loc[:, "pH"]

    ln_T_liquid.set_data(time, T_liquid)
    ln_T_cooler.set_data(time, T_cooler)
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


ani = FuncAnimation(figure, update, init_func=init, interval=1000 * 10)
plot.show()
