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
            d = np.loadtxt(data_path, skiprows=1)
            label = os.path.splitext(os.path.basename(data_path))[0]
            data.append([label, d])
    return data


def plot_arm_cost(data, ax):
    ax.set_title("Reward functions")
    ax.set_xlabel('x position (m)')
    ax.set_ylabel('function value')
    for label, d in data:
        x = d[:, 1]
        y = -d[:, 4]

        ax.plot(x, y, label=label + "_fvalue")

    ax.legend()
    ax.grid(True)


def plot_stability(data, ax):
    ax.set_title("Stability margin")
    ax.set_xlabel('x position (m)')
    ax.set_ylabel('FASM')
    for label, d in data:
        x = d[:, 1]
        y = d[:, 6]

        ax.plot(x, y, label=label + "_stability")

    ax.legend()
    ax.grid(True)


def main():
    args = parse_arguments()
    data = load_data(args.data_files)
    fig, axs = plt.subplots(2)
    plot_arm_cost(data, axs[0])
    plot_stability(data, axs[1])
    plt.show()


if __name__ == "__main__":
    main()
