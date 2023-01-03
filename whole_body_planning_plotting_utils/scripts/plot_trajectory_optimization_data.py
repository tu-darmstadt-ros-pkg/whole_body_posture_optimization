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


def get_next_color(fig, index):
    key = (fig, index)
    color_index = plot_color_index.get(key, -1) + 1
    plot_color_index[key] = color_index
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
                   line=get_next_color("lines", subplot_index)),
        row=subplot_index, col=1
    )

    fig.update_layout(scene=dict(
        xaxis_title="x",
        yaxis_title=col_name,
    ))


def compute_statistics(label, data, data_points, fig):
    print()
    print(label + ":")
    print("{:<18} {:<9} {:<9} {:<9}".format("Data", "Mean", "Min", "Max"))
    for index, data_point in enumerate(data_points):
        mean, minimum, maximum = compute_data_point_statistics(data_point, data[data_point])
        fig.add_trace(go.Bar(
            x=["Mean", "Minimum", "Maximum"],
            y=[mean, minimum, maximum],
            name=label + "-" + data_point,
            marker_color=get_next_color("bars", index + 1)["color"]
        ), row=index + 1, col=1)


def compute_data_point_statistics(data_label, data):
    mean = np.mean(data)
    minimum = np.min(data)
    maximum = np.max(data)

    print("{:<18} {:<9.4f} {:<9.4f} {:<9.4f}".format(data_label, mean, minimum, maximum))
    return mean, minimum, maximum


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
    statistics_data_points = ("cost", "optimization_time", "stability")
    fig_statistics = make_subplots(rows=3, cols=1, subplot_titles=statistics_data_points)

    for label, data in data_list:
        compute_statistics(label, data, statistics_data_points, fig_statistics)
        for index, key_value in enumerate(tables.items()):
            for data_point in key_value[1]:
                plot_data(data, label + "-" + data_point, fig, data_point, index+1)

    fig.show()
    fig_statistics.show()


if __name__ == "__main__":
    main()
