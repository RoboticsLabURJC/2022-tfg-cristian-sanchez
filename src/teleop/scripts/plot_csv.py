#!/usr/bin/env python
'''
Read the csv files, calculates the mean and plot it
'''

import csv
import numpy as np
import matplotlib.pyplot as plt

TIMES_PATH = '/home/csanrod/Desktop/tests_tfg/30x30/center (12,12)/times.csv'
IT_PATH = '/home/csanrod/Desktop/tests_tfg/30x30/center (12,12)/iterations.csv'
BAD_PATH = '/home/csanrod/Desktop/tests_tfg/30x30/center (12,12)/bad_moves.csv'

LABELS = ('Time (s)', 'Iterations', 'Bad moves (%)')
LABELS_EXP = ('Manual', 'Manual Opt', 'Q-Learning')

def get_avg_column_list(path):
    '''
    Return a list of float means of the csv file columns.
    '''
    avgs = []
    with open(path, 'r') as file:
        reader = csv.reader(file)
        data = list(reader)
        
        n_columns = len(data[0])

        for i in range(n_columns):
            nums = [float(row[i]) for row in data]
            avgs.append(np.mean(nums))

    return avgs


def plot_all(data):
    '''
    Plot avg from the csv files.
    '''
    rainbow_colors = ('blue', 'orange', 'green', 'red', 'indigo', 'yellow', 'violet')

    k = 0
    for i in range(len(LABELS)):
        variable = data[i]
        ax = plt.subplot(1, len(LABELS), i + 1)
        bars = plt.bar(LABELS_EXP, variable)

        ax.bar_label(bars)
        ax.set_ylabel(LABELS[i])

        if i == len(LABELS) - 1:
            ax.set_yticks(range(0, 101, 10))

        for j in range(len(bars)):
            try:
                bars[j].set_color(rainbow_colors[k%len(LABELS_EXP)])
            except IndexError:
                k = 0
                bars[j].set_color(rainbow_colors[k%len(LABELS_EXP)])

            k += 1

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    avgs_times = get_avg_column_list(TIMES_PATH)
    avgs_it = get_avg_column_list(IT_PATH)
    avgs_bad = get_avg_column_list(BAD_PATH)

    plot_all((avgs_times, avgs_it, avgs_bad))