#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on 2024 May 03 14:17

@author: Alexis Saldivar
"""
import pandas as pd
import Handle


sqlite_db = Handle.Database(DATABASE_PATH="database.db")
experiments_db = pd.read_csv("experiments_db.tsv", sep="\t")

if __name__ == "__main__":
    print(f"Dumping {len(experiments_db)} experments to CSVs")
    print("|\tname\t|\tdate\t|\tentries\t|")
    for i in experiments_db.index:
        name = experiments_db.loc[i, "experiment"]
        date = experiments_db.loc[i, "start_time"]
        query_str = f"SELECT * FROM {name}"
        data_df = pd.DataFrame(
            sqlite_db.cursor.execute(query_str)
        )
        entries = len(data_df)
        print(f"|\t{name}\t|\t{date}\t|\t{entries}\t|")
        data_df.to_csv(f"results/{name}.csv", index=False)

        experiments_db.loc[i, "entries"] = entries
    experiments_db.to_csv("experiments_db.tsv", sep="\t", index=False)
