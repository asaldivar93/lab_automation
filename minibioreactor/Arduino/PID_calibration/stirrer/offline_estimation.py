#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr  1 18:27:31 2020

@author: alexis
"""
from scipy.integrate import odeint
from scipy.optimize import minimize

import matplotlib.pyplot as plot
import pandas as pd
import numpy as np

parameters_df = pd.DataFrame(columns=["Gain", "Tau"])
parameters_df.loc[0, "Gain"] = 10   # Gain initial estimate
parameters_df.loc[0, "Tau"] = 1  # Response time initial estimate
iter = 0

# Initialize figure
plot.style.use('seaborn')
experiment = 'test_calibration'
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

    dydt = (-y + gain * u) / tau

    return dydt


def jacobian_fop(y, t, parameters, u):
    gain = parameters[0]  # process gain
    tau = parameters[1]  # Response Time
    J1 = (y - gain * u) / tau**2
    J2 = u / gain

    return [J1, J2]


def simulate(parameters):
    initial_conditions = data.loc[0, "RPM"]
    simulation_results = pd.DataFrame(columns=["Time", "u", "RPM"])
    simulation_results.loc[0, "Time"] = data.loc[0, "Time"]
    simulation_results.loc[0, "RPM"] = initial_conditions
    simulation_results.loc[0, "u"] = data.loc[0, "Stir_rate"]

    N = len(data)
    time = data.loc[0:, "Time"]
    u = data.loc[0:, "Stir_rate"]

    for i in range(1, N):
        time_interval = [time[i - 1], time[i]]
        solution = odeint(
            first_order_process, initial_conditions, time_interval, args=(parameters, u[i - 0])
        )

        simulation_results.loc[i, "Time"] = time[i]
        simulation_results.loc[i, "RPM"] = solution[-1][0]
        simulation_results.loc[i, "u"] = u[i]
        initial_conditions = solution[-1][0]

    return simulation_results


def squared_error(simulation_results):
    y = data.loc[:, "RPM"].to_numpy()
    yhat = simulation_results.loc[:, "RPM"].to_numpy()
    penalty = np.diag(0.8 * np.ones(len(data)))

    return np.dot(np.dot((y - yhat).transpose(), penalty), (y - yhat))


def model_deviation(simulation_results):
    yhat = simulation_results.loc[:, "RPM"].to_numpy()
    yhat_old = old_results.loc[:, "RPM"].to_numpy()
    penalty = np.diag(0.1 * np.ones(len(data)))

    return np.dot(np.dot((yhat - yhat_old).transpose(), penalty), (yhat - yhat_old))


def parameter_movement(parameters):
    p = np.array(parameters)
    p_old = parameters_df.iloc[-1, :].to_numpy()
    penalty = np.diag(0.1 * np.ones(len(p_old)))
    delta_p = (p - p_old)

    return np.dot(np.dot(delta_p.transpose(), penalty), delta_p)


def objective(parameters):
    simulation_results = simulate(parameters)

    objective = squared_error(simulation_results)

    return objective / len(data)


iter = iter + 1
data = pd.read_csv(experiment + '/data.csv').query("Vial==0")
data.loc[:, "Time"] = data.loc[:, "Time"] * 3600
data = data.query("Time<240").query("Time>30").tail(400).reset_index().drop(columns=['index'])
old_parameters = parameters_df.loc[iter - 1, :].to_list()
old_results = simulate(old_parameters)
new_parameters = minimize(objective, [1, 10], method='Nelder-Mead', tol=100)
parameters_df.loc[iter, :] = new_parameters.x

plot.plot(data.loc[:, "Time"], data.loc[:, "RPM"], old_results.loc[:, "Time"], old_results.loc[:, "RPM"], 'r')
plot.show()

plot.plot(parameters_df.index.to_list(), parameters_df.loc[:, "Gain"], "*")
plot.show()
