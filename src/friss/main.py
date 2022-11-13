#! /usr/bin/env python
import numpy as np
import matplotlib.pylab as plt
from matplotlib.widgets import Slider, Button

ROWS = 50
COLS = 50
SIGNAL_ORIGIN = (20, 20)
CUSTOM_RANGE = (-40, 40)

# Common frequency profiles
FREQ_WIFI = 2.4 * (10**9)
FREQ_5G = 30 * (10**9)
FREQ_FM = 100 * (10**6)

fig = plt.figure("Heatmap")
raw_data = np.empty((ROWS, COLS))

def scale_value(value, old_range, new_range):
    '''
    Transform a value in terms of [old_min, old_max] to an scaled value in terms of [new_min, new_max].
    '''
    old_min, old_max = old_range
    new_min, new_max = new_range

    norm = (value - old_min)/(old_max - old_min)
    new_value = (norm * (new_max - new_min)) + new_min

    return new_value

def scale_array(data, old_range, new_range):
    '''
    Transform data into the range we want.
    '''
    for x in range(ROWS):
        for y in range(COLS):
            data[x, y] = scale_value(data[x, y], old_range, new_range)
        
class Friss:
    C = 3.0 * (10 ** 8)
    DIST_FACTOR = 1.0

    def __init__(self, power_tras=1.0, gain_tras=1.0, gain_recv=1.0, freq=FREQ_WIFI, losses_factor=1.0, losses_path=2.0):
        self.__power_t = power_tras                     # Pt
        self.__gain_t = gain_tras                       # Gt
        self.__gain_r = gain_recv                       # Gr
        self.__lambda = self.__get_lambda(freq)         # Lambda
        self.__losses_f = losses_factor                 # L
        self.__losses_p = losses_path                   # n

    def set_values(self, power_tras, gain_tras, gain_recv, freq, losses_factor, losses_path):
        self.__power_t = power_tras                     # Pt
        self.__gain_t = gain_tras                       # Gt
        self.__gain_r = gain_recv                       # Gr
        self.__lambda = self.__get_lambda(freq)         # Lambda
        self.__losses_f = losses_factor                 # L
        self.__losses_p = losses_path                   # n

    def __get_lambda(self, freq):
        return self.C/freq

    def __friss_formula(self, dist):
        if dist == 0:
            dist = 0.9 * self.DIST_FACTOR

        power_recv = self.__power_t * (self.__gain_t * self.__gain_r * (self.__lambda)**2)/(((4 * np.pi)**2) * (dist**self.__losses_p) * self.__losses_f)
        power_recv_dBm = self.__W_to_dBm(power_recv)
        return power_recv_dBm

    def __propagation_path_loss(self, dist):
        if dist == 0:
            dist = 0.9 * self.DIST_FACTOR

        # Empty space case
        loss = -10 * np.log10((self.__lambda**2)/((4 * np.pi * dist)**2))   
        return loss

    def __del__(self):
        pass

    def model_power_signal(self, rows, cols, origin=(0,0)):
        global raw_data # For debugging

        x_origin, y_origin = origin
        data = np.zeros((rows, cols))
        for x in range(rows):
            for y in range(cols):
                dist_to_origin = (((x - x_origin)**2 + (y - y_origin)**2) ** (1/2)) * self.DIST_FACTOR
                pwr_recv = self.__friss_formula(dist_to_origin)
                data[x, y] = pwr_recv

                raw_data[x, y] = pwr_recv # For debugging

        current_range = (np.min(data), np.max(data))
        # scale_array(data, current_range, CUSTOM_RANGE)
        return data

    def model_signal_losses(self, rows, cols, origin=(0,0)):
        # NEED TO BE UPDATED!!!!!!!!!!!!!!
        x_origin, y_origin = origin
        data = np.zeros((rows, cols))
        for x in range(rows):
            for y in range(cols):
                dist_to_origin = (((x - x_origin)**2 + (y - y_origin)**2) ** (1/2)) * self.DIST_FACTOR
                losses = self.__propagation_path_loss(dist_to_origin)
                data[x, y] = losses

        norm_data = data / np.linalg.norm(data)
        self.__show_data(norm_data)
        # self.__show_data(data)

    def __W_to_dBm(self, value_in_w):
        dBm = 10 * np.log10(value_in_w / 0.001)
        return dBm

    def __W_to_dB(self, value_in_w):
        dB = 10 * np.log10(value_in_w)
        return dB

    def __rcv_power_with_ref(self, d_ref, d_final):
        # Calibrated
        power_recv = self.__friss_formula(d_ref) + 20 * np.log10(d_ref/d_final)
        return power_recv

    def __print_all(self):
        print("****************")
        print("Current Friss values:")
        print("\tPt:",self.__power_t, "W")
        print("\tGt:",self.__gain_t, "W")
        print("\tGr:",self.__gain_r, "W")
        print("\tLm:",self.__lambda)
        print("\tl:",self.__losses_f)
        print("\tn:",self.__losses_p)
        print("****************")
                 
    def show_data(self, data):
        plt.pcolormesh(data, cmap = 'winter')
        plt.title('Heatmap')
        plt.show()

    def test(self, dist, new_dist):
        global raw_data

        self.__print_all()
        print("P transmiter (", dist, "m ):", self.__W_to_dBm(self.__power_t), "dBm")
        print("P receiver (", dist, "m ):", self.__friss_formula(dist), "dBm")
        print("P transmiter (", new_dist, "m ):", self.__W_to_dBm(self.__power_t), "dBm")
        print("P receiver (", new_dist, "m ):", self.__friss_formula(new_dist), "dBm")

        current_range = (np.min(raw_data), np.max(raw_data))
        Pr = self.__friss_formula(new_dist)
        Pr_sc = scale_value(Pr, current_range, CUSTOM_RANGE)
        print("\nFor", new_dist,"m in dBm:")
        print("\tPr:", Pr,"in range", current_range)
        print("\tPr scaled:", Pr_sc, "in range", CUSTOM_RANGE)
        print("\n")

if __name__ ==  "__main__":
    my_model = Friss()

    data = my_model.model_power_signal(ROWS, COLS, SIGNAL_ORIGIN)
    my_model.test(2, 10)

    ax = fig.add_subplot(111)
    fig.subplots_adjust(bottom=0.4)
    mesh_min, mesh_max = CUSTOM_RANGE
    im = ax.imshow(data, cmap = 'viridis', aspect='equal', origin='lower', 
                    vmin=mesh_min, vmax=mesh_max)
    fig.colorbar(im)

    ax_Pt = fig.add_axes([0.15, 0.3, 0.65, 0.03])
    ax_Gt = fig.add_axes([0.15, 0.25, 0.65, 0.03])
    ax_Gr = fig.add_axes([0.15, 0.2, 0.65, 0.03])
    ax_Fq = fig.add_axes([0.15, 0.15, 0.65, 0.03])
    ax_l = fig.add_axes([0.15, 0.1, 0.65, 0.03])
    ax_n = fig.add_axes([0.15, 0.05, 0.65, 0.03]) 
    
    power_t = Slider(ax_Pt, 'Pt (W)', 0.01, 100.0, 1.0)
    gain_t = Slider(ax_Gt, 'Gt (W)', 0.01, 100.0, 1.0)
    gain_r = Slider(ax_Gr, 'Gr (W)', 0.01, 100.0, 1.0)
    freq = Slider(ax_Fq, 'Fq (GHz)', 0.01, 10.0, FREQ_WIFI/(10**9))
    losses_factor = Slider(ax_l, 'L', 0.01, 10.0, 1.0)
    loss_exp = Slider(ax_n, 'n', 1.6, 6.0, 2.0)    

    def update(val):
        global fig

        Pt = power_t.val
        Gt = gain_t.val
        Gr = gain_r.val
        Fq = freq.val * (10**9)
        l = losses_factor.val
        n = loss_exp.val

        my_model.set_values(Pt, Gt, Gr, Fq, l, n)
        data = my_model.model_power_signal(ROWS, COLS, SIGNAL_ORIGIN)

        mesh_min, mesh_max = CUSTOM_RANGE
        ax.imshow(data, cmap = 'viridis', aspect='equal', origin='lower', 
                  vmin=mesh_min, vmax=mesh_max)

        my_model.test(2, 10)
    
    power_t.on_changed(update)
    gain_t.on_changed(update)
    gain_r.on_changed(update)
    freq.on_changed(update)
    losses_factor.on_changed(update)
    loss_exp.on_changed(update)

    plt.show()
