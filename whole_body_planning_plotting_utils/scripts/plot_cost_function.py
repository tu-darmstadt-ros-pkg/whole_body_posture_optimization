#!/usr/bin/env python3
import plotly.express as px
import plotly.graph_objects as go
import numpy as np
import os
import argparse


def parse_arguments():
    parser = argparse.ArgumentParser(description='Plot cost function data')
    parser.add_argument('data_file', metavar='FILE', type=str,
                        help='data file to be processed')

    args = parser.parse_args()
    return args


def load_data(data_path):
    if os.path.isfile(data_path):
        data = np.genfromtxt(data_path, names=True, delimiter=",")
        return data
    else:
        return None


def plot_cost_function(data, fig, parameter: str, cost_function: str):
    fig.add_trace(go.Scatter(x=data[parameter], y=data[cost_function], name=cost_function, mode='markers'))

    fig.update_layout(
        xaxis_title=parameter,
        yaxis_title="cost",
    )


def plot_cost_function3d(data, fig, parameters: [], cost_function: str):
    fig.add_trace(go.Scatter3d(x=data[parameters[0]], y=data[parameters[1]], z=data[cost_function], name=cost_function, mode='markers'))

    fig.update_layout(scene=dict(
        xaxis_title=parameters[0],
        yaxis_title=parameters[1],
        zaxis_title="cost",))


def main():
    args = parse_arguments()
    data = load_data(args.data_file)
    if data is not None:
        fig = go.Figure()
        fig.update_layout(title="Cost function", scene=dict(
            camera=dict(
                projection=dict(
                    type="orthographic"
                )
            )
        ))

        parameters = ["flipper_front_joint", "flipper_back_joint"]
        # if len(parameters) > 1:
        #     plot_cost_function(data, fig, parameters[0], "track_contact_cost")
        #     plot_cost_function(data, fig, parameters[0], "stability_cost")
        #     plot_cost_function(data, fig, parameters[0], "total_cost")
        # else:
        plot_cost_function3d(data, fig, parameters, "track_contact_cost")
        plot_cost_function3d(data, fig, parameters, "stability_cost")
        plot_cost_function3d(data, fig, parameters, "total_cost")


        fig.show()
    else:
        print("Could not load data")


if __name__ == "__main__":
    main()
