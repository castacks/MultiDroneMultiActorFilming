#!/usr/bin/python3

import pandas as pd
import numpy as np

def summarize_experiment(df_root, expr_name: str):
    df_ppa = pd.read_csv(f"{expr_name}/ppa_evaluation.csv")
    df_image = pd.read_csv(f"{expr_name}/image_evaluation.csv")

    # print(df_ppa)
    for col_name in df_ppa.columns[1:]:
        median_ppa = np.median(df_ppa[col_name])
        median_image = np.median(df_image[col_name])
        sum_ppa = np.sum(df_ppa[col_name])
        sum_image = np.sum(df_image[col_name])
        # df_root["Median Image"][expr_name] = median_image
        # df_root["Median PPA"][expr_name] = median_ppa
        # df_root["Total Image"][expr_name] = sum_image
        # df_root["Total PPA"][expr_name] = sum_ppa
        df_root[col_name + "_" + "SRPPA"][expr_name] = median_ppa
        df_root[col_name + "_" + "Image"][expr_name] = median_image


def main():

    experiments = [
        "cluster",
        "cross_mix",
        "four_split",
        "priority_runners",
        "priority_speaker",
        "split_and_join",
        "spreadout_group",
        "track_runners",
        "heavy_mixing"
    ]

    planners = [
        "formation",
        "greedy",
        "assignment",
        "myopic",
        "multipleroundsgreedy",
    ]

    views = [
        "SRPPA",
        "Image",
    ]

    cols = []
    for pl in planners:
        for v in views:
            cols.append(pl + "_" + v)


    row_data = [0 for _ in cols]
    df_root = pd.DataFrame(data=[row_data for _ in experiments], columns=cols, index=experiments)
    for expr_name in experiments:
        summarize_experiment(df_root, expr_name)

    df_root.to_csv("total_summary.csv")
    print(df_root)

if __name__ == "__main__":
    main()
