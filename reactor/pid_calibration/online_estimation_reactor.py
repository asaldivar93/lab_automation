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

experiment = 'calibracion_reactor_20231109'

parameters_df = pd.DataFrame(columns=["UA_loss", "UA", "UA_loss1"])
parameters_df.loc[0, "UA_loss"] = 0.1049   # Heat losses to air
parameters_df.loc[0, "UA"] = 0.6539  # Heat transfer to liquid
parameters_df.loc[0, "UA_loss1"] = 0.09409
iter = 0
niter = 0
objective_val = [0]

results_file = experiment + "/parameters.csv"

with open(results_file, 'w+') as file:
    pd.DataFrame(
        columns=["U_loss", "U"],
    ).to_csv(file, index=False)

# Initialize figure
plot.style.use('seaborn')
figure = plot.figure(constrained_layout=True)
grid = figure.add_gridspec(6, 6)

# Temperature of the heater
ax_temperatures = figure.add_subplot(grid[:3, :4])
ln_T_heater, = ax_temperatures.plot([], [], "b.")
ln_T_heater_hat, = ax_temperatures.plot([], [], "r")
ln_T_liquid, = ax_temperatures.plot([], [], "g.")
ln_T_liquid_hat, = ax_temperatures.plot([], [], "y")

# UA
ax_UA = figure.add_subplot(grid[3:, :2])
ln_UA, = ax_UA.plot([], [], "*")

# UA_loss
ax_UA_loss = figure.add_subplot(grid[3:, 2:4])
ln_UA_loss, = ax_UA_loss.plot([], [], "*")

# UA_loss1
ax_UA_loss1 = figure.add_subplot(grid[3:, 4:])
ln_UA_loss1, = ax_UA_loss1.plot([], [], "*")

# Objective
ax_obj = figure.add_subplot(grid[:3, 4:])
ln_obj, = ax_obj.plot([], [], "*")


def electric_heater(T, t, parameters, u):
    Tr, TL = T
    # Parameters
    # U = global heat transfer coefficient
    # U_loss = heat losses to air
    # A = area of heat Transfer
    # T_amb = ambient temperature
    # L_mass = mass of the process liquid
    # gain = gain

    UA_loss = parameters[0]  # W / m^2 s
    UA = parameters[1]  # W / m^2 s
    UA_loss1 = parameters[2]
    gain = 0.1319
    T_amb = 24
    L_mass = 650
    Cp = 4.186  # J / g K
    mcpr = 153

    Qt = UA * (Tr - TL)
    Q_loss = UA_loss * (Tr - T_amb)
    Q_loss1 = UA_loss1 * (TL - T_amb)

    dTLdt = (Qt - Q_loss1) / (L_mass * Cp)
    dTrdt = (gain * u - (Qt + Q_loss)) / mcpr

    return [dTrdt, dTLdt]


def simulate(parameters):
    initial_conditions = data.loc[0, ["T_heater", "T_liquid"]].to_list()
    simulation_results = pd.DataFrame(columns=["Time", "u", "T_heater", "T_liquid"])
    simulation_results.loc[0, "Time"] = data.loc[0, "Time"]
    simulation_results.loc[0, "T_heater"] = initial_conditions[0]
    simulation_results.loc[0, "T_liquid"] = initial_conditions[1]
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
        simulation_results.loc[i, "T_heater"] = solution[-1, 0]
        simulation_results.loc[i, "T_liquid"] = solution[-1, 1]
        simulation_results.loc[i, "u"] = u[i]
        initial_conditions = [solution[-1, 0], solution[-1, 1]]

    return simulation_results


def squared_error(simulation_results):
    y = data.loc[:, ["T_heater", "T_liquid"]].to_numpy()
    yhat = simulation_results.loc[:, ["T_heater", "T_liquid"]].to_numpy()

    penalty = 0.8
    squared_error = np.sum(
        np.diag(
            np.dot((y - yhat).transpose(), (y - yhat))
        )
    )

    return penalty * squared_error


def model_deviation(simulation_results):
    yhat = simulation_results.loc[:, ["T_heater", "T_liquid"]].to_numpy()
    yhat_old = old_results.loc[:, ["T_heater", "T_liquid"]].to_numpy()
    penalty = 0.1
    deviation = np.sum(
        np.diag(
            np.dot((yhat - yhat_old).transpose(), (yhat - yhat_old))
        )
    )

    return penalty * deviation


def parameter_movement(parameters):
    p = np.array(parameters)
    p_old = parameters_df.iloc[-1, :].to_numpy()
    penalty = np.diag(0.1 * np.ones(len(p_old)))
    delta_p = (p - p_old)

    return penalty * np.dot(delta_p.transpose(), delta_p)


def objective(parameters):
    simulation_results = simulate(parameters)
    objective = squared_error(simulation_results) + model_deviation(simulation_results) + parameter_movement(parameters)

    return np.sum(objective) / len(data)


def init():
    ax_temperatures.set_ylim(20, 150)


# Data plot
def update(i):
    global data
    global old_results
    global parameters_df
    global iter
    global niter

    iter = iter + 1

    data = pd.read_csv(experiment + '/data.csv')
    data.loc[:, "Time"] = data.loc[:, "Time"] * 3600
    data = data.loc[::4].tail(900).reset_index().drop(columns=['index'])

    old_parameters = parameters_df.loc[iter - 1, :].to_list()
    old_results = simulate(old_parameters)

    new_parameters = minimize(
        objective, old_parameters, method='Nelder-Mead',
        bounds=[(0, None), (0, None), (0, None)], tol=0.1
    )
    parameters_df.loc[iter, :] = new_parameters.x
    objective_val.append(new_parameters.fun)

    with open(results_file, 'a+') as file:
        pd.DataFrame(parameters_df.iloc[-1, :]).transpose().to_csv(file, index=False, header=False)

    time = data.loc[:, "Time"] / 60
    T_heater = data.loc[:, "T_heater"]
    T_heater_hat = old_results.loc[:, "T_heater"]
    T_liquid = data.loc[:, "T_liquid"]
    T_liquid_hat = old_results.loc[:, "T_liquid"]
    UA = parameters_df.loc[:, "UA"]
    UA_loss = parameters_df.loc[:, "UA_loss"]
    UA_loss1 = parameters_df.loc[:, "UA_loss1"]

    ln_T_heater.set_data(time, T_heater)
    ln_T_heater_hat.set_data(time, T_heater_hat)
    ln_T_liquid.set_data(time, T_liquid)
    ln_T_liquid_hat.set_data(time, T_liquid_hat)
    ax_temperatures.relim()
    ax_temperatures.autoscale_view(scaley=False)

    ln_UA.set_data(parameters_df.index.to_list(), UA)
    ax_UA.relim()
    ax_UA.autoscale_view(scaley=True)

    ln_UA_loss.set_data(parameters_df.index.to_list(), UA_loss)
    ax_UA_loss.relim()
    ax_UA_loss.autoscale_view(scaley=True)

    ln_UA_loss1.set_data(parameters_df.index.to_list(), UA_loss1)
    ax_UA_loss1.relim()
    ax_UA_loss1.autoscale_view(scaley=True)

    ln_obj.set_data(parameters_df.index.to_list(), objective_val)
    ax_obj.relim()
    ax_obj.autoscale_view(scaley=True)


ani = FuncAnimation(figure, update, init_func=init, interval=1000 * 0.1)
plot.show()
