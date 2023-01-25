#! /usr/bin/env python
import numpy as np

# Common frequency profiles
FREQ_WIFI = 2.4 * (10**9)
FREQ_5G = 30 * (10**9)
FREQ_FM = 100 * (10**6)

class Friss:
    C = 3.0 * (10 ** 8)         # Light speed
    #DIST_FACTOR = 1.0           # Step distance, 1 = 1 m
    CUSTOM_RANGE = (-40, 40)    # Fixed maximun and minimun

    def __init__(self, power_tras=1.0, gain_tras=1.0, gain_recv=1.0, 
                 freq=FREQ_WIFI, losses_factor=1.0, losses_path=2.0, 
                 world_sz=(100, 100), resolution=1.0):

        self.__power_t = power_tras                     # Pt
        self.__gain_t = gain_tras                       # Gt
        self.__gain_r = gain_recv                       # Gr
        self.__lambda = self.__get_lambda(freq)         # Lambda
        self.__losses_f = losses_factor                 # L
        self.__losses_p = losses_path                   # n

        # Generar mundo con la resolución
        self.__resolution = resolution                  # units per cell, 0.5 means that every cell side measures 0.5 m
        self.__world_sz = world_sz                      # defines ticks in x, y (20, 20) is a 20 x 20 m map
        self.__raw_data = self.__generate_data_grid()   # data grid

    def __generate_data_grid(self):
        l_magnitude, _ = self.__world_sz
        l_grid = int(l_magnitude / self.__resolution)

        return np.empty((l_grid, l_grid))

    def reset_world(self, resolution, size):
        '''
        Resets current world to create a new one with new resolution and/or size.
        '''
        self.__resolution = resolution                  
        self.__world_sz = size                          
        self.__raw_data = self.__generate_data_grid()   

    def get_world_sz(self):
        '''
        Returns world size.
        '''
        return self.__world_sz

    def set_values(self, power_tras, gain_tras, gain_recv, freq, losses_factor, losses_path):
        '''
        Allows the modification of the friss formula parameters.
        '''
        self.__power_t = power_tras                     # Pt
        self.__gain_t = gain_tras                       # Gt
        self.__gain_r = gain_recv                       # Gr
        self.__lambda = self.__get_lambda(freq)         # Lambda
        self.__losses_f = losses_factor                 # L
        self.__losses_p = losses_path                   # n

    def __get_lambda(self, freq):
        '''
        Returns lambda.
        '''
        return self.C/freq

    def __friss_formula(self, dist):
        '''
        Returns received power in a certain distances in dBm.
        '''
        if dist == 0:
            dist = 0.9 * self.__resolution

        power_recv = self.__power_t * (self.__gain_t * self.__gain_r * (self.__lambda)**2)/(((4 * np.pi)**2) * (dist**self.__losses_p) * self.__losses_f)
        power_recv_dBm = self.__W_to_dBm(power_recv)
        return power_recv_dBm

    def __rcv_power_with_ref(self, d_ref, d_final):
        '''
        Obtain the received power with references.
        '''
        # Calibrated
        power_recv = self.__friss_formula(d_ref) + 20 * np.log10(d_ref/d_final)
        return power_recv

    def __propagation_path_loss(self, dist):
        '''
        Returns loss in a certain distances in dB.
        '''
        if dist == 0:
            dist = 0.9 * self.__resolution

        # Empty space case
        loss = -10 * np.log10((self.__lambda**2)/((4 * np.pi * dist)**2))   
        return loss

    def model_power_signal(self, origin=(0,0)):
        '''
        Fills data using friss formula and returns it.
        '''
        x_origin, y_origin = origin
        rows, cols = self.__raw_data.shape
        data = np.zeros((rows, cols))

        for x in range(rows):
            for y in range(cols):
                dist_to_origin = (((x - x_origin)**2 + (y - y_origin)**2) ** (1/2)) * self.__resolution
                pwr_recv = self.__friss_formula(dist_to_origin)
                data[x, y] = pwr_recv

                self.__raw_data[x, y] = pwr_recv # For debugging

        return data

    def model_signal_losses(self, origin=(0,0)):
        '''
        Fills data using propagation path loss formula and returns it.
        '''
        x_origin, y_origin = origin
        rows, cols = self.__raw_data.shape
        data = np.zeros((rows, cols))
        for x in range(rows):
            for y in range(cols):
                dist_to_origin = (((x - x_origin)**2 + (y - y_origin)**2) ** (1/2)) * self.DIST_FACTOR
                losses = self.__propagation_path_loss(dist_to_origin)
                data[x, y] = losses

                self.__raw_data[x, y] = losses # For debuggin

        return data

    # Revise!!
    def __W_to_dBm(self, value_in_w):
        '''
        Transform from W to dBm.
        '''
        dBm = 10 * np.log10(value_in_w / 0.001)
        return dBm

    def __W_to_dB(self, value_in_w):
        '''
        Transform from W to dB.
        '''
        dB = 10 * np.log10(value_in_w)
        return dB

    # -- DEBUG -- #
    def print_all(self):
        '''
        Shows every friss parameter used in the formulas.
        '''
        print("****************")
        print("Current Friss values:")
        print("\tPt:",self.__power_t, "W")
        print("\tGt:",self.__gain_t, "W")
        print("\tGr:",self.__gain_r, "W")
        print("\tLm:",self.__lambda)
        print("\tl:",self.__losses_f)
        print("\tn:",self.__losses_p)
        print("****************")
                 

    def test(self, dist, new_dist):
        '''
        Function used for testing purposes.
        '''
        self.print_all()
        print("P transmiter (", dist, "m ):", self.__W_to_dBm(self.__power_t), "dBm")
        print("P receiver (", dist, "m ):", self.__friss_formula(dist), "dBm")
        print("P transmiter (", new_dist, "m ):", self.__W_to_dBm(self.__power_t), "dBm")
        print("P receiver (", new_dist, "m ):", self.__friss_formula(new_dist), "dBm")

        current_range = (np.min(self.__raw_data), np.max(self.__raw_data))
        Pr = self.__friss_formula(new_dist)
        print("\nFor", new_dist,"m in dBm:")
        print("\tPr:", Pr,"in range", current_range)
        print("\n")

    def __del__(self):
        pass
