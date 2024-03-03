"""
Created on 2024 Mar 02 19:28

@author: Alexis Saldivar
"""

from collections import namedtuple
from datetime import datetime

import sqlite3
import os

import pandas as pd

EXPERIMENTS_DATABASE_PATH = "experiments_db.tsv"
inputs_columns_list = [("board", "TEXT"), ("experiment", "TEXT"),
                       ("type", "TEXT"), ("channel", "INT"), ("variable", "TEXT")]
datetime_iso = "%Y-%m-%d %H:%M:%-S"


class Database():
    def __init__(self, DATABASE_PATH: str = "database.db"):
        self.connection = sqlite3.connect(DATABASE_PATH)
        self.connection.row_factory = self.namedtuple_factory
        self.cursor = self.connection.cursor()
        self.set_inputs_table()

    def namedtuple_factory(self, cursor, row):
        fields = [column[0] for column in cursor.description]
        cls = namedtuple("Row", fields)
        return cls._make(row)

    def set_inputs_table(self):
        self.create_table("inputs", inputs_columns_list)

    def build_columns_definition_str(self, columns_definition_list: list) -> str:
        columns_definition = ""
        for name, type in columns_definition_list[:-1]:
            columns_definition = columns_definition + " ".join([name, type]) + ", "
        for name, type in [columns_definition_list[-1]]:
            columns_definition = columns_definition + " ".join([name, type])

        return columns_definition

    def create_table(self, table_name, columns_list):
        columns_definition_str = self.build_columns_definition_str(
            columns_list
        )
        if not self.is_table(table_name):
            sqlite_create = f"CREATE TABLE {table_name}({columns_definition_str})"
            self.cursor.execute(sqlite_create)

    def is_table(self, table_name: str) -> bool:
        sqlite_query = f"SELECT name FROM sqlite_master WHERE type='table' AND name='{table_name}'"
        return self.cursor.execute(sqlite_query).fetchone() is not None

    def insert_row(self, table_name, columns, values):
        sqlite_insert = f"INSERT INTO {table_name} ({columns}) VALUES ({values})"
        self.cursor.execute(sqlite_insert)
        self.connection.commit()


class Experiment():
    def __init__(self, name, sqlite_db, board):
        self.name = name
        self.sqlite_db = sqlite_db
        self.board = board
        if self.is_new_experiment():
            self.start_time = datetime.now()
            self.create_experiment_record()

    def is_new_experiment(self):
        if os.path.isfile(EXPERIMENTS_DATABASE_PATH):
            record = pd.read_csv(EXPERIMENTS_DATABASE_PATH, sep="\t", index_col="experiment")
            if self.name in record.index:
                return False
            else:
                return True
        else:
            return True

    def create_experiment_record(self):
        self.add_to_experiments_db()
        self.add_to_inputs_table()
        self.create_experiment_table()

    def add_to_experiments_db(self):
        record = pd.DataFrame(
            [[self.name, self.start_time.strftime(datetime_iso)]],
            columns=["experiment", "start_time"]
        )
        if os.path.isfile(EXPERIMENTS_DATABASE_PATH):
            with open(EXPERIMENTS_DATABASE_PATH, "a+") as file:
                record.to_csv(
                    file, index=False, header=False, sep="\t"
                )
        else:
            record.to_csv(
                EXPERIMENTS_DATABASE_PATH, index=False, sep="\t"
            )

    def add_to_inputs_table(self):
        table_name = "inputs"
        columns = "board, experiment, type, channel, variable"
        for input in self.board.inputs_list:
            values = f"'{self.board.ADDDRESS}', '{self.name}', 'input.type', {input.channel}, '{input.variable}'"
            self.sqlite_db.insert_row(table_name, columns, values)

    def create_experiment_table(self):
        table_name = self.name
        columns_list = self.build_experiment_columns_list()
        self.sqlite_db.create_table(table_name, columns_list)

    def build_experiment_columns_list(self):
        columns_list = [("date", "TEXT")]
        columns_list.extend(
            [("_".join([output.type, str(output.channel)]), "INT") for output in self.board.outputs_list]
        )
        columns_list.extend(
            [(input.variable, "REAL") for input in self.board.inputs_list]
        )
        return columns_list

    def save_data(self, time, data_dict):
        table_name = self.name
        time = time.isoformat(sep=" ", timespec="milliseconds")

        columns = ["date"]
        values = [f"'{time}'"]

        columns.extend(["_".join([type, str(channel)]) for type, channel, value in data_dict["outputs"]])
        values.extend([str(value) for type, channel, value in data_dict["outputs"]])

        columns.extend([variable for variable, value in data_dict["inputs"]])
        values.extend([str(value) for variable, value in data_dict["inputs"]])

        columns_str = ", ".join(columns)
        values_str = ", ".join(values)

        self.sqlite_db.insert_row(table_name, columns_str, values_str)
