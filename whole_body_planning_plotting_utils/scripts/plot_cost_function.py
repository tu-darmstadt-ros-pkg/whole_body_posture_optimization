#!/usr/bin/env python3
import matplotlib.pyplot as plt
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


def plot_cost_function(data, ax, parameter: str, cost_function: str):
    ax.set_title("Cost functions")
    ax.set_xlabel(parameter)
    ax.set_ylabel(cost_function)
    ax.scatter(data[parameter], data[cost_function], label=cost_function)

    ax.legend()
    ax.grid(True)


def main():
    args = parse_arguments()
    data = load_data(args.data_file)
    if data is not None:
        fig, axs = plt.subplots(1, 1)
        plot_cost_function(data, axs, "flipper_front_joint", "total_cost")
        plt.show()
    else:
        print("Could not load data")


if __name__ == "__main__":
    main()
