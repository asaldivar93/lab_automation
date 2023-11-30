#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# %% Codecell
# Environment setup
from datetime import datetime

import plotly.graph_objects as go
import plotly.express as pl
import plotly.io as pio

import pandas as pd

import json
import glob

with open('theme_template.json', 'r') as template:
    style = json.load(template)
style["layout"]["height"] = 2*61 * (600 / 158.75)
style["layout"]["width"] = 2*70 * (600 / 158.75)

pio.templates["paper"] = go.layout.Template(
    data=style["data"],
    layout=style["layout"]
)
pio.templates.default = "simple_white+paper"


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


# %% Codecell
# Data loading and preprocessing

# Loading
dir_pattern = "202311*_cinetica"
data_directories = glob.glob(dir_pattern)

data = pd.DataFrame()
for dir in data_directories:
    data = data.append(
        pd.read_csv(
            dir + "/data.csv"
        )
    )
data = data.reset_index().drop(columns=["index"])

# Experiment execution time calculation
data.loc[:, "Date"] = data.loc[:, "Date"].apply(
    string_to_datetime
    )

start_time = data.loc[0, "Date"]
data.loc[:, "Time"] = data.loc[:, "Date"].apply(
    get_time, args=(start_time, "hours",)
    )

# %% Codecell
# Plotting
sampling_time_seconds = 5
samples_per_minute = 60 / sampling_time_seconds
display_time_minutes = 0.5
filter_nth_element = int(samples_per_minute * display_time_minutes)
plot_data = data.iloc[::filter_nth_element, :]

fig_temperatures = go.Figure()
fig_temperatures.add_trace(
    go.Scatter(
        x=plot_data["Time"], y=plot_data["T_mosfet"],
        hovertext=plot_data["Date"],
        mode="lines", name="MOSFET"
    )
)
fig_temperatures.add_trace(
    go.Scatter(
        x=plot_data["Time"], y=plot_data["T_reactor"],
        hovertext=plot_data["Date"],
        mode="lines", name="Reactor"
    )
)
