"""
Created on 2024 Mar 02 19:28

@author: Alexis Saldivar
"""

from collections import namedtuple
from datetime import datetime

import sqlite3
import os

import pandas as pd

from Board import Dictlist

EXPERIMENTS_DATABASE_PATH = "experiments_db.tsv"
inputs_columns_list = [("board", "TEXT"), ("experiment", "TEXT"),
                       ("type", "TEXT"), ("channel", "INT"), ("id", "TEXT")]


class Database():
    def __init__(self, DATABASE_PATH: str = "database.db"):
        self.connection = sqlite3.connect(DATABASE_PATH, isolation_level=None)
        self.connection.execute('pragma journal_mode=wal')
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
    def __init__(self, name, sqlite_db, boards):
        self.name = name
        self.sqlite_db = sqlite_db
        self.set_boards(boards)
        if self.is_new_experiment():
            self.start_time = datetime.now()
            self.create_experiment_record()

    def set_boards(self, boards):
        self.boards = Dictlist(boards)

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
        # self.add_to_inputs_table()
        self.create_experiment_table()

    def add_to_experiments_db(self):
        record = pd.DataFrame(
            [[self.name, self.start_time.isoformat(sep=" ", timespec="milliseconds")]],
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
        columns = "board, experiment, type, channel, id"
        for input in self.board.Inputs:
            values = f"'{input.address}', '{self.name}', '{input.type}', {input.channel}, '{input.id}'"
            self.sqlite_db.insert_row(table_name, columns, values)

    def create_experiment_table(self):
        table_name = self.name
        columns_list = self.build_experiment_columns_list()
        self.sqlite_db.create_table(table_name, columns_list)

    def build_experiment_columns_list(self):
        columns_list = [("date", "TEXT")]

        for board in self.boards:
            columns_list.extend(
                [(output.id, "INTEGER") for output in board.Outputs]
            )

        for board in self.boards:
            columns_list.extend(
                [(input.id, "REAL") for input in board.Inputs]
            )
        return columns_list

    def save_data(self, time, data_dict):
        table_name = self.name
        time = time.isoformat(sep=" ", timespec="milliseconds")

        ids = ["date"]
        values = [f"'{time}'"]

        ids_to_add, values_to_add = self.build_sql_row_list(data_dict, "outputs")
        ids.extend(ids_to_add)
        values.extend(values_to_add)
        ids_to_add, values_to_add = self.build_sql_row_list(data_dict, "inputs")

        ids.extend(ids_to_add)
        values.extend(values_to_add)

        columns_str = ", ".join(ids)
        values_str = ", ".join(values)
        self.sqlite_db.insert_row(table_name, columns_str, values_str)

    def build_sql_row_list(self, data_dict, channel_type):
        ids_list = list()
        values_list = list()
        for address in data_dict.keys():
            board = self.boards.get_by_id(address)

            if channel_type == "inputs":
                channels_list = data_dict[address]["ins"]
            elif channel_type == "outputs":
                channels_list = data_dict[address]["outs"]

            for chn, val in channels_list:
                if channel_type == "inputs":
                    channel = board.Inputs.get_by_channel("_".join([address, str(chn)]))
                elif channel_type == "outputs":
                    channel = board.Outputs.get_by_channel("_".join([address, str(chn)]))
                ids_list.append(channel.id)
                values_list.append(str(val))

        return ids_list, values_list
