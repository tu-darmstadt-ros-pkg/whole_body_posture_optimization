#!/usr/bin/env python3
import argparse
import numpy as np
import os
import plotly.express as px
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import plotly


def parse_arguments():
    parser = argparse.ArgumentParser(description='Plot trajectory data')
    parser.add_argument('data_files', metavar='FILES', type=str, nargs='+',
                        help='data files to be processed')

    args = parser.parse_args()
    return args


def load_data(data_files):
    data = []
    for data_path in data_files:
        if os.path.isfile(data_path):
            d = np.genfromtxt(data_path, names=True, delimiter=",")
            label = os.path.splitext(os.path.basename(data_path))[0]
            data.append([label, d])
    return data


plot_color_index = dict()


def get_next_color(index):
    color_index = plot_color_index.get(index, -1) + 1
    plot_color_index[index] = color_index
    palette = plotly.colors.DEFAULT_PLOTLY_COLORS
    color_index = color_index % len(palette)
    return dict(width=2, color=palette[color_index])


def plot_data(data, label, fig, col_name, subplot_index):
    # x = np.arange(0, len(data[col_name]), 1.0)
    if np.isnan(data[col_name]).any() or (data[col_name] == 0).all():
        return
    fig.add_trace(
        go.Scatter(x=data["pose_x"],
                   y=data[col_name],
                   name=label,
                   line=get_next_color(subplot_index)),
        row=subplot_index, col=1
    )

    fig.update_layout(scene=dict(
        xaxis_title="x",
        yaxis_title=col_name,
    ))


def main():
    args = parse_arguments()
    data_list = load_data(args.data_files)

    tables = {
        "cost": [
            # "custom_solver_cost",
            "cost"
        ],
        "iterations": [
            # "custom_solver_iterations",
            "iterations"
        ],
        "optimization_time": [
            "optimization_time"
        ],
        "stability": [
            "stability"
        ]
    }

    fig = make_subplots(rows=len(tables.keys()), cols=1, subplot_titles=tuple(tables.keys()))
    for label, data in data_list:
        for index, key_value in enumerate(tables.items()):
            for data_point in key_value[1]:
                plot_data(data, label + "-" + data_point, fig, data_point, index+1)

    fig.show()


if __name__ == "__main__":
    main()
