# %%codecell
from datetime import datetime

from scipy.integrate import odeint
from scipy.optimize import minimize
from scipy.linalg import expm

import pandas as pd
import numpy as np
import json

import plotly.graph_objects as go
import plotly.express as pl
import plotly.io as pio

with open('theme_template.json', 'r') as template:
    style = json.load(template)
style["layout"]["height"] = 2 * 61 * (600 / 158.75)
style["layout"]["width"] = 2 * 70 * (600 / 158.75)

pio.templates["paper"] = go.layout.Template(
    data=style["data"],
    layout=style["layout"]
)
pio.templates.default = "simple_white+paper"


# %%codecell

def string_to_datetime(date_string, date_format="%d-%m-%Y_%H-%M-%S"):
    date = datetime.strptime(date_string, date_format)

    return date


def get_time(date, start_date, units):
    time_delta = date - start_date
    time_delta_seconds = time_delta.days * 24 * 3600 + time_delta.seconds
    time_delta_minutes = time_delta_seconds / 60
    time_delta_hours = time_delta_seconds / 3600
    time_delta_days = time_delta_hours / 24

    if units == "seconds":
        return time_delta_seconds
    if units == "minutes":
        return time_delta_minutes
    if units == "hours":
        return time_delta_hours
    if units == "days":
        return time_delta_days


def odeint_heater(T, t, parameters, u):
    T_heater, T_liquid, T_sensor = T
    # Parameters

    Kh = parameters[0]
    UAh = parameters[1]
    UAs = parameters[2]
    UAloss_h = parameters[3]
    UAloss_l = parameters[4]
    mcps = parameters[5]
    mcph = 153
    mcpl = 6279
    PWM_heater = u[0]
    T_amb = u[1]

    Q_gain = Kh * PWM_heater
    Q_transfer = UAh * (T_heater - T_liquid)
    Q_loss_h = UAloss_h * (T_heater - T_amb)
    Q_sensor = UAs * (T_liquid - T_sensor)
    Q_loss_l = UAloss_l * (T_liquid - T_amb)

    dT_heater = (Q_gain - (Q_transfer + Q_loss_h)) / mcph
    dT_liquid = (Q_transfer - (Q_sensor + Q_loss_l)) / mcpl
    dT_sensor = Q_sensor / mcps

    return [dT_heater, dT_liquid, dT_sensor]


def odeint_heater_2(T, t, parameters, u):
    T_heater, T_liquid = T
    # Parameters

    Kh = parameters[0]
    UAh = parameters[1]
    UAloss_h = parameters[2]
    UAloss_l = parameters[3]
    mcph = 153
    mcpl = 6279
    PWM_heater = u[0]
    T_amb = u[1]

    Q_gain = Kh * PWM_heater
    Q_transfer = UAh * (T_heater - T_liquid)
    Q_loss_h = UAloss_h * (T_heater - T_amb)
    Q_loss_l = UAloss_l * (T_liquid - T_amb)

    dT_heater = (Q_gain - (Q_transfer + Q_loss_h)) / mcph
    dT_liquid = (Q_transfer - Q_loss_l) / mcpl

    return [dT_heater, dT_liquid]


def statespace_heater2(T, t, parameters, U):
    T = np.array([T[0], T[1]])
    U = np.array([U[0], U[1]])
    a11 = parameters[0]
    a12 = parameters[1]
    a21 = parameters[2]
    a22 = parameters[3]
    b11 = parameters[4]
    b12 = parameters[5]
    b22 = parameters[6]

    A = np.array([[a11, a12],
                  [a21, a22]])
    B = np.array([[b11, b12],
                  [0, b22]])
    F = expm(A * t)
    Tk = np.dot(F, T) + np.dot(B, U)

    return list(Tk)


def statespace_heater(T, t, parameters, U):
    T_heater, T_liquid = T
    a11 = parameters[0]
    a12 = parameters[1]
    a21 = parameters[2]
    a22 = parameters[3]
    b11 = parameters[4]
    b12 = parameters[5]
    b22 = parameters[6]

    dT_heater = a11 * T_heater + a12 * T_liquid + b11 * U[0] + b12 * U[1]
    dT_liquid = a21 * T_heater + a22 * T_liquid + b22 * U[1]

    return [dT_heater, dT_liquid]


def simulate(parameters):
    initial_conditions = fit_data.loc[0, ["T_heater", "T_sensor"]].to_list()
    simulation_results = pd.DataFrame(columns=["Time", "T_heater", "T_sensor"])
    simulation_results.loc[0, "Time"] = fit_data.loc[0, "Time"]
    simulation_results.loc[0, "T_heater"] = initial_conditions[0]
    simulation_results.loc[0, "T_sensor"] = initial_conditions[1]

    N = len(fit_data)
    time = fit_data.loc[:, "Time"]

    for i in range(1, N):
        time_interval = [time[i - 1], time[i]]
        solution = odeint(
            odeint_heater_2, initial_conditions, time_interval, args=(parameters, u[i - 1])
        )

        simulation_results.loc[i, "Time"] = time[i]
        simulation_results.loc[i, "T_heater"] = solution[-1, 0]
        simulation_results.loc[i, "T_sensor"] = solution[-1, 1]
        initial_conditions = [solution[-1, 0], solution[-1, 1]]

    return simulation_results


def simulate_statespace(parameters):
    initial_conditions = fit_data.loc[0, ["T_heater", "T_sensor"]].to_list()
    simulation_results = pd.DataFrame(columns=["Time", "T_heater", "T_sensor"])
    simulation_results.loc[0, "Time"] = fit_data.loc[0, "Time"]
    simulation_results.loc[0, "T_heater"] = initial_conditions[0]
    simulation_results.loc[0, "T_sensor"] = initial_conditions[1]

    N = len(fit_data)
    time = fit_data.loc[:, "Time"]

    for i in range(1, N):
        time_interval = [time[i - 1], time[i]]
        solution = odeint(
            statespace_heater, initial_conditions, time_interval, args=(parameters, u[i - 1])
        )
        simulation_results.loc[i, "Time"] = time[i]
        simulation_results.loc[i, "T_heater"] = solution[-1, 0]
        simulation_results.loc[i, "T_sensor"] = solution[-1, 1]
        initial_conditions = [solution[-1, 0], solution[-1, 1]]

    return simulation_results


def simulate_statespace2(parameters):
    initial_conditions = fit_data.loc[0, ["T_heater", "T_sensor"]].to_list()
    simulation_results = pd.DataFrame(columns=["Time", "T_heater", "T_sensor"])
    simulation_results.loc[0, "Time"] = fit_data.loc[0, "Time"]
    simulation_results.loc[0, "T_heater"] = initial_conditions[0]
    simulation_results.loc[0, "T_sensor"] = initial_conditions[1]

    N = len(fit_data)
    time = fit_data.loc[:, "Time"]

    for i in range(1, N):
        time_interval = [time[i - 1], time[i]]
        solution = runge_kutta4(
            initial_conditions, time_interval, 0.001, statespace_heater, parameters, u[i - 1]
        )
        simulation_results.loc[i, "Time"] = time[i]
        simulation_results.loc[i, "T_heater"] = solution[-1, 0]
        simulation_results.loc[i, "T_sensor"] = solution[-1, 1]
        initial_conditions = [solution[-1, 0], solution[-1, 1]]

    return simulation_results


def squared_error(simulation_results):
    y = fit_data.loc[:, ["T_heater", "T_sensor"]].to_numpy()
    yhat = simulation_results.loc[:, ["T_heater", "T_sensor"]].to_numpy()
    squared_error = np.sum(
        np.diag(
            np.dot((y - yhat).transpose(), (y - yhat))
        )
    )

    return squared_error


def objective(parameters):
    simulation_results = simulate(parameters)
    objective = squared_error(simulation_results)

    return objective


def objective_statespase(parameters):
    simulation_results = simulate_statespace(parameters)
    objective = squared_error(simulation_results)

    return objective


def runge_kutta4(T, t, h, f, parameters, U):
    """computes 4th order Runge-Kutta for dy/dx.
    y is the initial value for y
    x is the initial value for x
    dx is the difference in x (e.g. the time step)
    f is a callable function (y, x) that you supply
    to compute dy/dx for the specified values.
    """
    T = np.array(T)
    iters = int((t[1] - t[0]) / h + 1)
    y = np.zeros((iters + 1, len(T)))
    y[0] = T
    x = t[0]
    for iter in range(0, iters):
        k1 = h * np.array(f(y[iter], x, parameters, U))
        k2 = h * np.array(f(y[iter] + 0.5 * k1, x + 0.5 * h, parameters, U))
        k3 = h * np.array(f(y[iter] + 0.5 * k2, x + 0.5 * h, parameters, U))
        k4 = h * np.array(f(y[iter] + k3, x + h, parameters, U))
        y[iter + 1] = y[iter] + (k1 + 2 * k2 + 2 * k3 + k4) / 6.
        x = x + h

    return y


def simulate_statespace3(parameters):
    initial_conditions = fit_data.loc[0, ["T_heater", "T_sensor"]].to_list()
    simulation_results = pd.DataFrame(columns=["Time", "T_heater", "T_sensor"])
    simulation_results.loc[0, "Time"] = fit_data.loc[0, "Time"]
    simulation_results.loc[0, "T_heater"] = initial_conditions[0]
    simulation_results.loc[0, "T_sensor"] = initial_conditions[1]

    N = len(fit_data)
    time = fit_data.loc[:, "Time"]

    for i in range(1, N):
        time_interval = [time[i - 1], time[i]]
        solution = runge_kutta4(
            initial_conditions, time_interval, 0.01, statespace_heater, parameters, u[i - 1]
        )
        simulation_results.loc[i, "Time"] = time[i]
        simulation_results.loc[i, "T_heater"] = solution[-1, 0]
        simulation_results.loc[i, "T_sensor"] = solution[-1, 1]
        initial_conditions = [solution[-1, 0], solution[-1, 1]]

    return simulation_results


def statespace_solver(Y0, t, dt, F, B, U):
    Y0 = np.array(Y0)
    iters = int((t[1] - t[0]) / dt + 1)
    y = np.zeros((iters + 1, len(Y0)))
    y[0] = Y0

    for iter in range(0, iters):
        y[iter + 1] = np.dot(F, y[iter]) + np.dot(B, U) * dt
    return y


def simulate_statespace4(parameters):
    a11 = parameters[0]
    a12 = parameters[1]
    a21 = parameters[2]
    a22 = parameters[3]
    b11 = parameters[4]
    b12 = parameters[5]
    b22 = parameters[6]

    A = np.array([[a11, a12],
                  [a21, a22]])
    B = np.array([[b11, b12],
                  [0, b22]])
    dt = 0.05
    D, E = np.linalg.eig(A)
    eD = np.diag(
        np.exp(D * dt)
    )
    F = np.dot(
        np.dot(E, eD), np.linalg.inv(E)
    )
    initial_conditions = fit_data.loc[0, ["T_heater", "T_sensor"]].to_list()
    simulation_results = pd.DataFrame(columns=["Time", "T_heater", "T_sensor"])
    simulation_results.loc[0, "Time"] = fit_data.loc[0, "Time"]
    simulation_results.loc[0, "T_heater"] = initial_conditions[0]
    simulation_results.loc[0, "T_sensor"] = initial_conditions[1]

    N = len(fit_data)
    time = fit_data.loc[:, "Time"]

    for i in range(1, N):
        time_interval = [time[i - 1], time[i]]
        solution = statespace_solver(
            initial_conditions, time_interval, dt, F, B, u[i - 1]
        )
        simulation_results.loc[i, "Time"] = time[i]
        simulation_results.loc[i, "T_heater"] = solution[-1, 0]
        simulation_results.loc[i, "T_sensor"] = solution[-1, 1]
        initial_conditions = [solution[-1, 0], solution[-1, 1]]

    return simulation_results


# %%codecell
data = pd.read_csv(
    "20240108_calibracion_pid_temp/data.csv"
)
data.query("PWM_heater<255").loc[:, ["T_heater", "T_liquid", "T_sensor", "T_amb"]].std()

# %%codecell
data = pd.read_csv(
    "20240108_calibracion_pid_temp2/data.csv"
)
data = data.drop(index=[0, 1]).reset_index()
data.loc[:, "Time"] = data.index * 0.250
measurements = ["T_heater", "T_sensor"]

fit_data = data.query("Time<=10000").loc[::16, :].reset_index()
u = [[fit_data.loc[i, "PWM_heater"], fit_data.loc[i, "T_amb"]] for i in fit_data.index]

# %%codecell
parameters = pd.DataFrame(
    columns=["Kh", "UAh", "UAloss_h", "UAloss_l"]
)
parameters.loc[0, "Kh"] = 0.28
parameters.loc[0, "UAh"] = 3
parameters.loc[0, "UAloss_h"] = 1.065
parameters.loc[0, "UAloss_l"] = 0.001

simulation = simulate(parameters.loc[0, :].to_list())

# %%codecell

new_parameters = minimize(
    objective, parameters.loc[0, :].to_list(),
    bounds=[(0, None), (3, None), (0.4, None), (0, None)],
    method="Nelder-Mead", tol=100
)
parameters.loc[1, :] = new_parameters.x
simulation = simulate(parameters.loc[1, :].to_list())
# %%codecell

plot_data = fit_data.iloc[:, :]

fig_temperatures = go.Figure()
for sensor in measurements:
    fig_temperatures.add_trace(
        go.Scatter(
            x=plot_data["Time"], y=plot_data[sensor],
            mode="markers", name=sensor
        )
    )

for sensor in measurements:
    fig_temperatures.add_trace(
        go.Scatter(
            x=simulation["Time"], y=simulation[sensor],
            mode="lines", name=sensor
        )
    )
fig_temperatures.show()

# %%codecell
parameters
a11 = -(3 + 0.9839) / 153
a12 = 3 / 153
a21 = 3 / 6279
a22 = -(3 + 0.0011) / 6279
b11 = 0.26 / 153
b12 = 0.9839 / 153
b21 = 0
b22 = 0.0011 / 153
parameters = [a11, a12, a21, a22, b11, b12, b22]
simulation = simulate_statespace(parameters)

new_parameters = minimize(
    objective_statespase, parameters,
    bounds=[(-1, 0), (0, None),
            (0, None), (-1, 0),
            (0, None), (0, None), (0, None)],
    method="Nelder-Mead", tol=100
)
simulation = simulate_statespace(list(new_parameters.x))
simulation = simulate_statespace3(list(new_parameters.x))
simulation = simulate_statespace4(list(parameters))

a11 = -2.60555105e-02
a12 = 1.93486978e-02
a21 = 4.79818914e-04
a22 = -4.77109910e-04
b11 = 1.74989411e-03
b12 = 6.33628509e-03
b22 = 7.20916773e-06

# %%codecell


def simulate_control(parameters, kp, ki):
    a11 = parameters[0]
    a12 = parameters[1]
    a21 = parameters[2]
    a22 = parameters[3]
    b11 = parameters[4]
    b12 = parameters[5]
    b22 = parameters[6]

    A = np.array([[a11, a12],
                  [a21, a22]])
    B = np.array([[b11, b12],
                  [0, b22]])
    dt_s = 0.01
    D, E = np.linalg.eig(A)
    eD = np.diag(
        np.exp(D * dt_s)
    )
    F = np.dot(
        np.dot(E, eD), np.linalg.inv(E)
    )

    initial_conditions = [25., 25.]
    simulation_results = pd.DataFrame(
        columns=["Time", "T_heater", "T_liquid", "u", "P", "I"]
    )
    simulation_results.loc[0, "Time"] = 0
    simulation_results.loc[0, "T_heater"] = initial_conditions[0]
    simulation_results.loc[0, "T_liquid"] = initial_conditions[1]
    simulation_results.loc[0, "T_heater_f"] = initial_conditions[0]
    simulation_results.loc[0, "T_liquid_f"] = initial_conditions[0]
    simulation_results.loc[0, "u"] = 0

    time_span = 10000
    dt = 0.250
    iters = int((time_span / dt) + 1)
    time = np.linspace(0, time_span, iters)
    I = np.zeros(iters)
    u = np.zeros(iters)
    sys_noise_filter = np.zeros([2,2])
    sys_noise_filter[0] = initial_conditions
    alpha_filter = 0.01
    for iter in range(1, iters):
        time_interval = [0, dt]
        sys_update = statespace_solver(
            initial_conditions, time_interval, dt_s, F, B, [u[iter - 1], 22]
        )
        sys_noise = sys_update[-1] + np.array([np.random.normal(0, 0.25), np.random.normal(0, 0.25)])
        sys_noise_filter[1] = alpha_filter * sys_noise + (1 - alpha_filter) * sys_noise_filter[0]
        input = sys_noise_filter[1, 1]
        error = 50 - input
        P = kp * error
        I[iter] = I[iter - 1] + ki * error * dt
        u[iter] = np.ceil(P + I[iter])
        if u[iter] > 255:
            u[iter] = 255
            I[iter] = I[iter - 1]
        if u[iter] < 0:
            u[iter] = 0
            I[iter] = I[iter - 1]
        sys_noise_filter[0] = sys_noise_filter[1]
        simulation_results.loc[iter, "Time"] = time[iter]
        simulation_results.loc[iter, "T_heater"] = sys_noise[0]
        simulation_results.loc[iter, "T_liquid"] = sys_noise[1]
        simulation_results.loc[iter, "T_heater_f"] = sys_noise_filter[1, 0]
        simulation_results.loc[iter, "T_liquid_f"] = sys_noise_filter[1, 1]
        simulation_results.loc[iter, "P"] = P
        simulation_results.loc[iter, "I"] = I[iter]
        simulation_results.loc[iter, "u"] = u[iter]

        initial_conditions = [sys_update[-1, 0], sys_update[-1, 1]]

    return simulation_results

# %%codecell
control = simulate_control(parameters, 100, 0.2)
pl.line(control, x="Time", y="u")
pl.line(control, x="Time", y=["T_liquid", "T_liquid_f"])
control.loc[:, "T_liquid"] = control.loc[:, "T_liquid"].astype(float)


















# %%codecell
np.linspace(0,0,0)
