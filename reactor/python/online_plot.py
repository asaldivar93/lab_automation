#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr  1 18:27:31 2020

@author: alexis
"""
from datetime import datetime

from matplotlib.animation import FuncAnimation
from sqlite3 import OperationalError

import matplotlib.pyplot as plot
import pandas as pd

import Handle


class Plotter:
    def __init__(self, experiment_name: str, DATABASE_PATH: str = "database.db", time_units: str = "hours"):
        self.db_path = DATABASE_PATH
        self.experiment = experiment_name
        self.time_units = time_units
        self.sqlite_db = self.connect_to_db(DATABASE_PATH)
        self.data_df = pd.DataFrame()
        self.start_time = self.get_startime()

    def connect_to_db(self, DATABASE_PATH):
        return Handle.Database(DATABASE_PATH="database.db")

    def get_startime(self):
        query_str = f"SELECT * FROM {self.experiment} WHERE ROWID = 1"
        data_df = pd.DataFrame(
            self.sqlite_db.cursor.execute(query_str)
        )
        data_df.loc[:, "date"] = data_df.loc[:, "date"].apply(
            lambda date_str: datetime.fromisoformat(date_str)
        )
        return data_df.loc[0, "date"]

    def get_data(self, time_span: float = 24):
        query_str = f"SELECT * FROM {self.experiment} WHERE date >= datetime('now', 'localtime', '-24 {self.time_units}') AND ROWID % 4 = 0"
        try:
            data_df = pd.DataFrame(
                self.sqlite_db.cursor.execute(query_str)
            )
        except OperationalError:
            self.sqlite_db.connection.close()
            self.sqlite_db = self.connect_to_db(self.db_path)

        return data_df

    def create_figure(self):
        figure = plot.figure(constrained_layout=True)
        grid = figure.add_gridspec(4, 4)
        return figure, grid

    def create_axes(self, figure, grid):
        self.ax_00 = figure.add_subplot(grid[:2, :2])
        ln_00_0, = self.ax_00.plot([], [], " g.")
        ln_00_1, = self.ax_00.plot([], [], "b.")
        ln_00_2, = self.ax_00.plot([], [], "r.")
        ln_00_3, = self.ax_00.plot([], [], "c.")
        self.ax_00_ln = [ln_00_0, ln_00_1, ln_00_2, ln_00_3]

        self.ax_01 = figure.add_subplot(grid[:2, 2:])
        ln_01_0, = self.ax_01.plot([], [], " g.")
        ln_01_1, = self.ax_01.plot([], [], "b.")
        ln_01_2, = self.ax_01.plot([], [], "r.")
        ln_01_3, = self.ax_01.plot([], [], "c.")
        self.ax_01_ln = [ln_01_0, ln_01_1, ln_01_2, ln_01_3]

        self.ax_10 = figure.add_subplot(grid[2:, :2])
        ln_10_0, = self.ax_10.plot([], [], " g.")
        ln_10_1, = self.ax_10.plot([], [], "b.")
        ln_10_2, = self.ax_10.plot([], [], "r.")
        ln_10_3, = self.ax_10.plot([], [], "c.")
        self.ax_10_ln = [ln_10_0, ln_10_1, ln_10_2, ln_10_3]

        self.ax_11 = figure.add_subplot(grid[2:, 2:])
        ln_11_0, = self.ax_11.plot([], [], " g.")
        ln_11_1, = self.ax_11.plot([], [], "b.")
        ln_11_2, = self.ax_11.plot([], [], "r.")
        ln_11_3, = self.ax_11.plot([], [], "c.")
        self.ax_11_ln = [ln_11_0, ln_11_1, ln_11_2, ln_11_3]

    def set_ax_00_data(self, time, variables: list):
        for i in range(0, len(variables)):
            data_y = self.data_df.loc[:, variables[i]]
            self.ax_00_ln[i].set_data(time, data_y)

    def set_ax_01_data(self, time, variables: list):
        for i in range(0, len(variables)):
            data_y = self.data_df.loc[:, variables[i]]
            self.ax_01_ln[i].set_data(time, data_y)

    def set_ax_10_data(self, time, variables: list):
        for i in range(0, len(variables)):
            data_y = self.data_df.loc[:, variables[i]]
            self.ax_10_ln[i].set_data(time, data_y)

    def set_ax_11_data(self, time, variables: list):
        for i in range(0, len(variables)):
            data_y = self.data_df.loc[:, variables[i]]
            self.ax_11_ln[i].set_data(time, data_y)

    def set_ax_00_vars(self, variables: list):
        self.ax_00_vars = variables

    def set_ax_01_vars(self, variables: list):
        self.ax_01_vars = variables

    def set_ax_10_vars(self, variables: list):
        self.ax_10_vars = variables

    def set_ax_11_vars(self, variables: list):
        self.ax_11_vars = variables

    def init(self):
        self.ax_00.autoscale_view(scaley=True)
        self.ax_01.autoscale_view(scaley=True)
        self.ax_10.autoscale_view(scaley=True)
        self.ax_11.autoscale_view(scaley=True)

    def update(self, i):
        self.data_df = self.get_data(time_span=24)
        self.data_df.loc[:, "date"] = self.data_df.loc[:, "date"].apply(
            lambda date_str: datetime.fromisoformat(date_str)
        )
        start_time = self.start_time
        time = self.data_df.loc[:, "date"].apply(
            self.get_time, args=(start_time, self.time_units,)
        )

        self.set_ax_00_data(time, self.ax_00_vars)
        self.set_ax_01_data(time, self.ax_01_vars)
        self.set_ax_10_data(time, self.ax_10_vars)
        self.set_ax_11_data(time, self.ax_11_vars)

        self.ax_00.autoscale_view(scaley=True)
        self.ax_01.autoscale_view(scaley=True)
        self.ax_10.autoscale_view(scaley=True)
        self.ax_11.autoscale_view(scaley=True)

        self.ax_00.relim()
        self.ax_01.relim()
        self.ax_10.relim()
        self.ax_11.relim()

    def get_time(self, date, start_date, units):
        time_delta = date - start_date
        time_delta_seconds = time_delta.days * 24 * 3600 + time_delta.seconds + time_delta.microseconds / 1e6

        if units == "seconds":
            return time_delta_seconds
        if units == "minutes":
            return time_delta_seconds / 60
        if units == "hours":
            return time_delta_seconds / 3600
        if units == "days":
            return time_delta_seconds / 3600 / 24


if __name__ == "__main__":
    plotter = Plotter(experiment_name="test", time_units="seconds")

    plotter.set_ax_00_vars(["temperature_0", "temperature_1"])
    plotter.set_ax_01_vars(["M0pwm_0"])
    plotter.set_ax_10_vars(["dissolved_oxygen"])
    plotter.set_ax_11_vars(["ph"])

    figure, grid = plotter.create_figure()
    plotter.create_axes(figure, grid)

    ani = FuncAnimation(figure, plotter.update,
                        init_func=plotter.init, interval=1000 * 1,
                        cache_frame_data=False)
    plot.show()
