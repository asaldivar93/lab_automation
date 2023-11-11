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

experiment = 'calibracion_resistencia_20231109_1'

parameters_df = pd.DataFrame(columns=["U_loss", "Gain"])
parameters_df.loc[0, "U_loss"] = 12.72345699   # Heat losses to air
parameters_df.loc[0, "Gain"] = 0.12901237  # Heat gain from electric current
iter = 0
objective_val = [0]

results_file = experiment + "/parameters.csv"

with open(results_file, 'w+') as file:
    pd.DataFrame(
        columns=["U_loss", "Gain"],
    ).to_csv(file, index=False)

# Initialize figure
plot.style.use('seaborn')
figure = plot.figure(constrained_layout=True)
grid = figure.add_gridspec(4, 4)

# Temperature of the heater
ax_rpm = figure.add_subplot(grid[:2, :2])
ln_rpm, = ax_rpm.plot([], [])
ln_rpm_hat, = ax_rpm.plot([], [], "r")

# Gain
ax_gain = figure.add_subplot(grid[:2, 2:])
ln_gain, = ax_gain.plot([], [], "*")

# U_loss
ax_tau = figure.add_subplot(grid[2:, :2])
ln_tau, = ax_tau.plot([], [], "*")

# Objective
ax_obj = figure.add_subplot(grid[2:, 2:])
ln_obj, = ax_obj.plot([], [], "*")


def electric_heater(T, t, parameters, u):
    # Parameters
    # U_loss = heat losses to air
    # A = area of heat Transfer
    # T_amb = ambient temperature
    # gain = gain

    U_loss = parameters[0]  # W / m^2 s
    gain = parameters[1]  #
    A = 0.05346  # m^2
    T_amb = 24.93  # Celsius
    R_mass = 212.4  # grams
    Cpr = 0.710  # J / g K

    Q_loss = A * U_loss * (T - T_amb)
    dTrdt = (gain * u - Q_loss) / (R_mass * Cpr)

    return dTrdt


def simulate(parameters):
    initial_conditions = data.loc[0, "T_heater"]
    simulation_results = pd.DataFrame(columns=["Time", "u", "T_heater"])
    simulation_results.loc[0, "Time"] = data.loc[0, "Time"]
    simulation_results.loc[0, "T_heater"] = initial_conditions
    simulation_results.loc[0, "u"] = data.loc[0, "PWM"]

    N = len(data)
    time = data.loc[0:, "Time"]
    u = data.loc[0:, "PWM"]

    for i in range(1, N):
        time_interval = [time[i - 1], time[i]]
        solution = odeint(
            electric_heater, initial_conditions, time_interval, args=(parameters, u[i - 1])
        )

        simulation_results.loc[i, "Time"] = time[i]
        simulation_results.loc[i, "T_heater"] = solution[-1][0]
        simulation_results.loc[i, "u"] = u[i]
        initial_conditions = solution[-1][0]

    return simulation_results


def squared_error(simulation_results):
    y = data.loc[:, "T_heater"].to_numpy()
    yhat = simulation_results.loc[:, "T_heater"].to_numpy()
    penalty = np.diag(0.8 * np.ones(len(data)))

    return np.dot(np.dot((y - yhat).transpose(), penalty), (y - yhat))


def model_deviation(simulation_results):
    yhat = simulation_results.loc[:, "T_heater"].to_numpy()
    yhat_old = old_results.loc[:, "T_heater"].to_numpy()
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
    ax_rpm.set_ylim(20, 70)


# Data plot
def update(i):
    global data
    global old_results
    global parameters_df
    global iter

    iter = iter + 1
    print(iter)	
    data = pd.read_csv(experiment + '/data.csv')
    data.loc[:, "Time"] = data.loc[:, "Time"] * 3600
    data = data.loc[::2].reset_index().drop(columns=['index'])

    old_parameters = parameters_df.loc[iter - 1, :].to_list()
    old_results = simulate(old_parameters)

    new_parameters = minimize(objective, old_parameters, method='Nelder-Mead', bounds=[(0, None), (0, None)], tol=1)
    parameters_df.loc[iter, :] = new_parameters.x
    objective_val.append(new_parameters.fun)

    with open(results_file, 'a+') as file:
        pd.DataFrame(parameters_df.iloc[-1, :]).transpose().to_csv(file, index=False, header=False)

    time = data.loc[:, "Time"]
    rpm = data.loc[:, "T_heater"]
    rpm_hat = old_results.loc[:, "T_heater"]
    gain = parameters_df.loc[:, "Gain"]
    tau = parameters_df.loc[:, "U_loss"]

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


ani = FuncAnimation(figure, update, init_func=init, interval=1000 * 30)
plot.show()
