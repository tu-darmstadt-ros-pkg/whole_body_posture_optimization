#!/usr/bin/env python3
import plotly.graph_objects as go
import numpy as np
import os
import argparse

import plotly.subplots


def parse_arguments():
    parser = argparse.ArgumentParser(description='Plot cost function PDF')
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


def plot_pdf(data_sorted, fig):
    # bin sum
    bin_size = 0.05
    bins = np.arange(data_sorted["flipper_cost"][0], data_sorted["flipper_cost"][-1], bin_size)
    cost_bin_indices = np.digitize(data_sorted["flipper_cost"], bins)
    cost_binned = np.bincount(cost_bin_indices, data_sorted["rel_probability"])

    # fig.add_trace(
    #     go.Scatter(x=data["flipper_cost"],
    #                y=data["rel_probability"],
    #                name="PDF",
    #                mode='markers')
    # )

    # fig.add_trace(
    #     go.Bar(x=bins,
    #            y=cost_binned[1:],
    #            name="PDF",)
    # )

    fig.add_trace(
        go.Histogram(
            x=data_sorted["flipper_cost"],
            y=data_sorted["rel_probability"],
            xbins=dict(start=data_sorted["flipper_cost"][0], end=data_sorted["flipper_cost"][-1], size=bin_size),
            histfunc="sum",
            histnorm='probability density'
        )
    )
    fig.update_xaxes(tickvals=list(bins.round(2)))

    fig.update_layout(
        xaxis_title="Cost",
        yaxis_title="Relative prob.",
    )


def plot_cost_over_pose(data, fig):
    x_column = "pose_x"

    fig.add_trace(
        go.Bar(x=data[x_column],
               y=data["flipper_cost"],
               name="Cost")
    )

    scaling = np.abs(np.max(data["flipper_cost"]) / np.max(data["rel_probability"]))
    fig.add_trace(
        go.Scatter(x=data[x_column],
                   y=data["rel_probability"] * scaling,
                   name="Rel. prob.")
    )

    fig.update_layout(
        xaxis_title=x_column,
        yaxis_title="Cost",
    )


def compute_statistics(data, fig):
    # Cost of mean pose
    max_prob_index = np.argmax(data["rel_probability"])
    mean_pose_cost = data["flipper_cost"][max_prob_index]
    fig.add_vline(x=mean_pose_cost, annotation_text="mean pose", annotation_position="top right")
    print("Cost of mean pose:", mean_pose_cost)

    # Mean cost
    mean = np.average(data["flipper_cost"], weights=data["rel_probability"])
    fig.add_vline(x=mean, annotation_text="mean", annotation_position="top left")
    print("Mean cost", mean)

    # Var and cVar
    compute_cvar(data, 0.5, fig)


def compute_cvar(data_sorted, risk_level, fig):
    """

    :param data_sorted:
    :param risk_level: 0-1, 0 (high risk), 1 (conservative)
    :param fig:
    """
    # Cumulative probability (not normalized)
    rel_prob_cum_sum = np.cumsum(data_sorted["rel_probability"])
    # Part that we risk
    part = risk_level * rel_prob_cum_sum[-1]
    # Value at risk
    start_index = np.argmax(rel_prob_cum_sum > part)
    var = data_sorted["flipper_cost"][start_index]
    fig.add_vline(x=var, annotation_text="var" + str(risk_level), annotation_position="top left")
    print("Value-at-risk:", var)
    # Conditional value at risk
    cvar = np.average(data_sorted["flipper_cost"][start_index:], weights=data_sorted["rel_probability"][start_index:])
    fig.add_vline(x=cvar, annotation_text="cvar-" + str(risk_level), annotation_position="top right")
    print("Conditional Value-at-risk:", cvar)


def main():
    args = parse_arguments()
    data = load_data(args.data_file)
    filename = os.path.basename(args.data_file)
    if data is not None:
        fig2 = go.Figure()
        fig2.update_layout(title="Cost Plot - " + filename)
        plot_cost_over_pose(data, fig2)
        fig2.show()

        fig = go.Figure()
        fig.update_layout(title="Probability Density Function - " + filename)
        # Sort by cost
        data_sorted = np.sort(data, order="flipper_cost")
        plot_pdf(data_sorted, fig)
        compute_statistics(data_sorted, fig)
        fig.show()
    else:
        print("Could not load data")


if __name__ == "__main__":
    main()
