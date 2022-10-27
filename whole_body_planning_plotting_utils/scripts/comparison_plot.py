#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import os
import argparse


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
            d = np.genfromtxt(data_path, names=True)
            label = os.path.splitext(os.path.basename(data_path))[0]
            data.append([label, d])
    return data


def plot_stability(data, ax):
    ax.set_title("Stability margin")
    ax.set_xlabel('x position (m)')
    ax.set_ylabel('FASM')

    # plot data
    for label, d in data:
        x = d["pos_x"]
        y = d["stability"]

        ax.plot(x, y, label=label)

    ax.legend()
    ax.grid(True)


def print_mean_stability(data):
    print("Average stability:")
    for label, d in data:
        print("-", label, np.mean(d["stability"]))


def print_min_stability(data):
    print("Minimum stability:")
    for label, d in data:
        print("-", label, np.min(d["stability"]))


def main():
    args = parse_arguments()
    data = load_data(args.data_files)
    fig, ax = plt.subplots(1, 1)
    plot_stability(data, ax)
    print_mean_stability(data)
    print_min_stability(data)

    plt.show()


if __name__ == "__main__":
    main()
