#!/usr/bin/env python3
import argparse
import numpy as np
import os
import plotly.express as px
import plotly.graph_objects as go
from plotly.subplots import make_subplots


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


def plot_data(data, label, fig, col_name, subplot_index):
    x = np.arange(0, len(data[col_name]), 1.0)
    fig.add_trace(
        go.Scatter(x=x, y=data[col_name], name=label),
        row=subplot_index, col=1
    )

    fig.update_layout(scene=dict(
        xaxis_title="Index",
        yaxis_title=col_name,
    ))


def main():
    args = parse_arguments()
    data_list = load_data(args.data_files)

    data_points = ("cost", "iterations", "optimization_time")
    fig = make_subplots(rows=len(data_points), cols=1, subplot_titles=data_points)
    for label, data in data_list:
        for index, data_point in enumerate(data_points):
            plot_data(data, label + "-" + data_point, fig, data_point, index+1)

    fig.show()


if __name__ == "__main__":
    main()
