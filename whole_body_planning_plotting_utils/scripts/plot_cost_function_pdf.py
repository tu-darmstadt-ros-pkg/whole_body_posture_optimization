#!/usr/bin/env python3
import plotly.graph_objects as go
import numpy as np
import os
import argparse


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


def plot_pdf(data, fig):
    # Sort by cost
    data.sort(order="flipper_cost")

    fig.add_trace(
        go.Scatter(x=data["flipper_cost"],
                   y=data["rel_probability"],
                   name="PDF",
                   mode='markers')
    )


def main():
    args = parse_arguments()
    data = load_data(args.data_file)
    filename = os.path.basename(args.data_file)
    if data is not None:
        fig = go.Figure()
        plot_pdf(data, fig)
        fig.show()
    else:
        print("Could not load data")


if __name__ == "__main__":
    main()
