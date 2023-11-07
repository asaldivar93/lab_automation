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

experiment = 'test_calibration'

parameters_df = pd.DataFrame(columns=["Gain", "Tau"])
parameters_df.loc[0, "Gain"] = 10   # Gain initial estimate
parameters_df.loc[0, "Tau"] = 1    # Response time initial estimate
iter = 0
objective_val = [0]

results_file = experiment + "/parameters.csv"

with open(results_file, 'w+') as file:
    pd.DataFrame(
        columns=['Gain', 'Tau'],
    ).to_csv(file, index=False)

# Initialize figure
plot.style.use('seaborn')
experiment = 'test_calibration'
figure = plot.figure(constrained_layout=True)
grid = figure.add_gridspec(4, 4)

ax_rpm = figure.add_subplot(grid[:2, :2])
ln_rpm, = ax_rpm.plot([], [])
ln_rpm_hat, = ax_rpm.plot([], [], "r")

ax_gain = figure.add_subplot(grid[:2, 2:])
ln_gain, = ax_gain.plot([], [], "*")

ax_tau = figure.add_subplot(grid[2:, :2])
ln_tau, = ax_tau.plot([], [], "*")

ax_obj = figure.add_subplot(grid[2:, 2:])
ln_obj, = ax_obj.plot([], [], "*")


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
            first_order_process, initial_conditions, time_interval,
            Dfun=jacobian_fop, args=(parameters, u[i - 0])
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

    objective = squared_error(simulation_results) + model_deviation(simulation_results) + parameter_movement(parameters)

    return objective / len(data)


def init():
    ax_rpm.set_ylim(500, 1700)


# Data plot
def update(i):
    global data
    global old_results
    global parameters_df
    global iter

    iter = iter + 1

    data = pd.read_csv(experiment + '/data.csv').query("Vial==0")
    data.loc[:, "Time"] = data.loc[:, "Time"] * 3600

    data = data.query("Time<240").query("Time>30").tail(400).reset_index().drop(columns=['index'])

    old_parameters = parameters_df.loc[iter - 1, :].to_list()
    old_results = simulate(old_parameters)

    new_parameters = minimize(objective, old_parameters, method='Nelder-Mead', tol=100)
    parameters_df.loc[iter, :] = new_parameters.x
    objective_val.append(new_parameters.fun)

    with open(results_file, 'a+') as file:
        pd.DataFrame(parameters_df.iloc[-1, :]).transpose().to_csv(file, index=False, header=False)

    time = data.loc[:, "Time"]
    rpm = data.loc[:, "RPM"]
    rpm_hat = old_results.loc[:, "RPM"]
    gain = parameters_df.loc[:, "Gain"]
    tau = parameters_df.loc[:, "Tau"]

    ln_rpm.set_data(time, rpm)
    ln_rpm_hat.set_data(time, rpm_hat)
    ax_rpm.relim()
    ax_rpm.autoscale_view(scaley=False)

    ln_gain.set_data(parameters_df.index.to_list(), gain)
    ax_gain.relim()
    ax_gain.autoscale_view(scaley=True)

    ln_tau.set_data(parameters_df.index.to_list(), tau)
    ax_tau.relim()
    ax_tau.autoscale_view(scaley=True)

    ln_obj.set_data(parameters_df.index.to_list(), objective_val)
    ax_obj.relim()
    ax_obj.autoscale_view(scaley=True)


ani = FuncAnimation(figure, update, init_func=init, interval=1000 * 10)
plot.show()
