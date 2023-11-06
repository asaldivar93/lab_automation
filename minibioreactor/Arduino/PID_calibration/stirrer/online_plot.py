#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr  1 18:27:31 2020

@author: alexis
"""
from matplotlib.animation import FuncAnimation
from scipy.integrate import odeint
from scipy.optimize import minimize

import matplotlib.pyplot as plot
import pandas as pd
import numpy as np

# Initialize figure
plot.style.use('seaborn')
experiment = 'calibration'
figure = plot.figure(constrained_layout=True)
grid = figure.add_gridspec(4, 4)

ax_do = figure.add_subplot(grid[:2, :2])  # Plot OD  # Plot ODS
ax_ph = figure.add_subplot(grid[:2, 2:])
ax_atm = figure.add_subplot(grid[2:, :])
ln_do, = ax_do.plot([], [])              # Plot ODS
ln_ph, = ax_ph.plot([], [])
ln_atm, = ax_atm.plot([], [])


def first_order_process(y, t, parameters, u):
    # Parameters
    gain = parameters[0]  # process gain
    tau = parameters[1]  # Response Time
    delay = parameters[2]  # Lag time

    dydt = (-y + gain * u * (t - delay)) / tau

    return dydt


def simulate(parameters):
    initial_conditions = data.loc[1, "RPM"]
    simulation_results = pd.DataFrame(columns=["Time", "u", "RPM"])
    simulation_results.loc[0, "Time"] = data.loc[1, "Time"]
    simulation_results.loc[0, "RPM"] = initial_conditions
    simulation_results.loc[0, "u"] = data.loc[1, "Stir_rate"]

    N = len(data)
    time = data.loc[1:, "Time"]
    u = data.loc[1:, "Stir_rate"]

    for i in range(1, N):
        time_interval = [time[i - 1], time[i]]
        solution = odeint(
            first_order_process, initial_conditions, time_interval, args=(parameters, u[i - 1])
        )

        simulation_results.loc[i, "Time"] = time[i]
        simulation_results.loc[i, "RPM"] = solution[-1]
        simulation_results.loc[i, "u"] = u[i]
        initial_conditions = solution[-1]

    return simulation_results


def squared_error(simulation_results):
    y = data[:, "RPM"].to_numpy()
    yhat = simulation_results[:, "RPM"].to_numpy()
    penalty = np.diag(0.5 * np.ones(len(data)))

    return np.dot(np.dot((y - yhat).transpose(), penalty), (y - yhat))


def model_deviation(simulation_results):
    yhat = simulation_results[:, "RPM"].to_numpy()
    yhat_old = old_results[:, "RPM"].to_numpy()
    penalty = np.diag(0.5 * np.ones(len(data)))

    return np.dot(np.dot((yhat - yhat_old).transpose(), penalty), (yhat - yhat_old))


def objective(parameters):
    simulation_results = simulate(parameters)

    objective = squared_error(simulation_results) + model_deviation(simulation_results)

    return objective


def init():
    # ax_ODF.set_yscale('log', subsy = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9,1])
    ax_do.set_ylim(0, 100)
    ax_ph.set_ylim(6, 8)
    ax_atm.set_ylim(0.65, 0.8)


# Data plot
def update(i):
    global data
    global old_results
    data = pd.read_csv(experiment + '/data.csv')

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

data = pd.DataFrame([1 * np.ones(30), 1.5 * np.ones(30)]).transpose()
y = data.iloc[:, 0].to_numpy()
yhat = data.iloc[:, 1].to_numpy()
penalty = 0.5 * np.ones(30)
np.dot(np.dot((y - yhat).transpose(), np.diag(penalty)), (y - yhat))

data.iloc[-1, :].to_list()
