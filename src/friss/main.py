#! /usr/bin/env python
import numpy as np
import matplotlib.pylab as plt
from matplotlib.widgets import Slider, Button

ROWS = 50
COLS = 50
SIGNAL_ORIGIN = (20, 20)
CUSTOM_RANGE = (0, 50)

# Common frequency profiles
FREQ_WIFI = 2.4 * (10**9)
FREQ_5G = 30 * (10**9)
FREQ_FM = 100 * (10**6)

def scale_value(value, org_range, dst_range):
    '''
    Transform a value in terms of [min, max] to an scaled value in terms of [a, b].
    '''
    min, max = org_range
    a, b = dst_range    

    factor = (b - a)/(max - min)
    scaled_value = factor * (value - min) + a

    return scaled_value

def scale_array(data, org_range, dst_range):
    '''
    Transform data into the range we want.
    '''
    for x in range(ROWS):
        for y in range(COLS):
            data[x, y] = scale_value(data[x, y], org_range, dst_range)
        
class Friss:
    C = 3.0 * (10 ** 8)
    DIST_FACTOR = 1.0

    def __init__(self, power_tras, gain_tras, gain_recv, freq=900.0 * (10 ** 6), losses_factor=1.0, losses_path=2.0):
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
        x_origin, y_origin = origin
        data = np.zeros((rows, cols))
        for x in range(rows):
            for y in range(cols):
                dist_to_origin = (((x - x_origin)**2 + (y - y_origin)**2) ** (1/2)) * self.DIST_FACTOR
                pwr_recv = self.__friss_formula(dist_to_origin)
                data[x, y] = pwr_recv

        current_range = (np.min(data), np.max(data))
        scale_array(data, current_range, CUSTOM_RANGE)
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
        self.__print_all()
        print("Pt (", dist, "m ):", self.__W_to_dBm(self.__power_t), "dBm")
        print("Pt (", dist, "m ):", self.__W_to_dB(self.__power_t), "dB")
        print("Pr (", dist, "m ):", self.__friss_formula(dist), "dBm")
        print("Pr (", new_dist, "m ), without friss:", self.__rcv_power_with_ref(dist, new_dist), "dBm")
        print("Pr (", new_dist, "m ), with friss:", self.__friss_formula(new_dist), "dBm")
        print("\n")

if __name__ ==  "__main__":
    plt.figure("Heatmap")

    Pt = 1.0
    Gt = 1.0
    Gr = 1.0
    Fq = 250.0 * (10**6)
    l = 1.0
    n = 2.0

    my_model = Friss(Pt, Gt, Gr, Fq, l, n)

    my_model.test(2, 10)
    # my_model.test(10, 100)

    data = my_model.model_power_signal(ROWS, COLS, SIGNAL_ORIGIN)

    ax = plt.subplot(211)
    plot = plt.pcolormesh(data, cmap = 'viridis')
    plt.colorbar(plot)

    ax_Pt = plt.axes([0.25, 0.3, 0.65, 0.03])
    ax_Gt = plt.axes([0.25, 0.25, 0.65, 0.03])
    ax_Gr = plt.axes([0.25, 0.2, 0.65, 0.03])
    ax_Fq = plt.axes([0.25, 0.15, 0.65, 0.03])
    ax_l = plt.axes([0.25, 0.1, 0.65, 0.03])
    ax_n = plt.axes([0.25, 0.05, 0.65, 0.03])    
    
    power_t = Slider(ax_Pt, 'Pt (W)', 0.0, 100.0, Pt)
    gain_t = Slider(ax_Gt, 'Gt (W)', 0.0, 100.0, Gt)
    gain_r = Slider(ax_Gr, 'Gr (W)', 0.0, 100.0, Gr)
    freq = Slider(ax_Fq, 'Fq (Hz)', 0, 10**10, Fq)
    losses_factor = Slider(ax_l, 'L', 0.0, 10.0, l)
    loss_exp = Slider(ax_n, 'n', 1.6, 6.0, n)    

    def update(val):
        Pt = power_t.val
        Gt = gain_t.val
        Gr = gain_r.val
        Fq = freq.val
        l = losses_factor.val
        n = loss_exp.val

        my_model.set_values(Pt, Gt, Gr, Fq, l, n)
        data = my_model.model_power_signal(ROWS, COLS, SIGNAL_ORIGIN)
        plot = ax.pcolormesh(data, cmap = 'viridis')
        plt.colorbar(plot)

        my_model.test(2, 10)
        # my_model.test(10, 100)
    
    power_t.on_changed(update)
    gain_t.on_changed(update)
    gain_r.on_changed(update)
    freq.on_changed(update)
    losses_factor.on_changed(update)
    loss_exp.on_changed(update)

    # manager = plt.get_current_fig_manager()
    # manager.full_screen_toggle()

    plt.show()
