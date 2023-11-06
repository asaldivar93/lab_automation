#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr  1 18:27:31 2020

@author: alexis
"""

import matplotlib.pyplot as plot
from matplotlib.animation import FuncAnimation
import numpy as np

# Initialize figure
plot.style.use('seaborn')
experiment='Biofiltro0902'
figure = plot.figure(constrained_layout=True)
grid = figure.add_gridspec(4, 4)

ax_do = figure.add_subplot(grid[:2, :2])  # Plot OD  # Plot ODS
ax_ph = figure.add_subplot(grid[:2, 2:])
ax_atm = figure.add_subplot(grid[2:, :])
ln_do, = ax_do.plot([], [])              # Plot ODS
ln_ph, = ax_ph.plot([], [])
ln_atm, = ax_atm.plot([], [])


def init():
    # ax_ODF.set_yscale('log', subsy = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9,1])
    ax_do.set_ylim(0, 100)
    ax_ph.set_ylim(6, 8)
    ax_atm.set_ylim(0.65, 0.8)

#Data plot
def update(i):
    data = np.loadtxt(experiment + '/data.csv', delimiter =',', skiprows = 1, usecols=(1,2,3,5)) ############# Nombre
    Time = data[:,0]
    ph = data[:,1]
    do = data[:,2]
    atm = data[:,3]
    
    ln_do.set_data(Time, do)
    ax_do.relim()
    ax_do.autoscale_view(scaley = False)

    ln_ph.set_data(Time, ph)
    ax_ph.relim()
    ax_ph.autoscale_view(scaley = False)

    ln_atm.set_data(Time, atm)
    ax_atm.relim()
    ax_atm.autoscale_view(scaley = False)


ani = FuncAnimation(figure, update, init_func = init, interval = 500)
plot.show()
