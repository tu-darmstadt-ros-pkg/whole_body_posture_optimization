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
        d = np.loadtxt(data_path, skiprows=1)
        label = os.path.splitext(os.path.basename(data_path))[0]
        data.append([label, d])
    return data


def plot_arm_cost(data):
    plt.figure()
    plt.title("Reward functions")
    plt.xlabel('x position (m)')
    plt.ylabel('function value')
    for label, d in data:
        x = d[:, 1]
        y = -d[:, 4]

        plt.plot(x, y, label=label)

    plt.legend()
    plt.grid(True)


def plot_stability(data):
    plt.figure()
    plt.title("Stability margin")
    plt.xlabel('x position (m)')
    plt.ylabel('FASM')
    for label, d in data:
        x = d[:, 1]
        y = d[:, 6]

        plt.plot(x, y, label=label)

    plt.legend()
    plt.grid(True)


def main():
    args = parse_arguments()
    data = load_data(args.data_files)
    plot_arm_cost(data)
    plot_stability(data)
    plt.show()


if __name__ == "__main__":
    main()
