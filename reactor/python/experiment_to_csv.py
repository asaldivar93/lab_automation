#!/usr/bin/env python3
"""Saves experiment data to csv.

Created on 2024 May 15 19:25.
@author: Alexis Saldivar
"""
from typing import object

import Handle
import argparse
import polars as pl

sqlite_db = Handle.Database(DATABASE_PATH="database.db")
experiments_db = pd.read_csv("experiments_db.tsv", sep="\t")


def argument_parser(args=None) -> object:
    parser = argparse.ArgumentParser(description="Exports experiment data to .csv")
    parser.add_argument("-n", "--name", type=str, help="Name of the experiment")

    return parser.parse_args()


if __name__ == "__main__":
    arguments = argument_parser()
    experiment = arguments.name
    print(f"Dumping {experiment} experments to csv")

    query_str = f"SELECT * FROM {experiment}"
    data_df = pd.DataFrame(
        sqlite_db.cursor.execute(query_str)
    )
    entries = len(data_df)
    print(f"Experiment {experiment} has {entries} entries")
    data_df.to_csv(f"results/{experiment}.csv", index=False)
