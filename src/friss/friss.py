#! /usr/bin/env python
'''
FRISS

With this module, you are able to simulate a radio frequency
signal, by using the friss theory.
'''
import numpy as np

# Common frequency profiles
FREQ_WIFI = 2.4 * (10**9)
FREQ_5G = 5 * (10**9)
FREQ_FM = 100 * (10**6)

class Friss:
    '''
    Here is a summary of the API:

        reset_world         --> Resets current world.
        get_world_sz        --> Returns world size.
        set_values          --> Set friss formula parameters.
        model_power_signal  --> Fills power data.
        model_signal_losses --> Fills losses data.
        print_all           --> Prints all parameters.
        test                --> Model some cases.
    '''
    C = 3.0 * (10 ** 8) # Light speed
    OBSTACLE_VALUE = -999

    def __init__(self, power_tras=1.0, gain_tras=1.0, gain_recv=1.0,
                 freq=FREQ_WIFI, losses_factor=2.4, losses_path=2.0,
                 world_sz=(50, 50), resolution=1.0):
        '''
        Constructor. Here it's filled all parameters to create a Friss model.
        '''

        self.__power_t = power_tras                     # Pt
        self.__gain_t = gain_tras                       # Gt
        self.__gain_r = gain_recv                       # Gr
        self.__lambda = self.__get_lambda(freq)         # Lambda
        self.__losses_f = losses_factor                 # L
        self.__losses_p = losses_path                   # n

        self.__resolution = resolution # units per cell, 0.5 =cell side is 0.5 m
        self.__world_sz = world_sz # defines ticks in x, y (20, 20) = 20x20 map
        self.__raw_data = self.__generate_data_grid() # data grid
        self.__signal_origin = (0,0)


    def __generate_data_grid(self):
        '''
        Creates a grid with empty data to fill.
        '''
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
        self.__signal_origin = origin
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
                dist_to_origin = (((x - x_origin)**2 + (y - y_origin)**2) ** (1/2)) * self.__resolution
                losses = self.__propagation_path_loss(dist_to_origin)
                data[x, y] = losses

                self.__raw_data[x, y] = losses # For debuggin

        return data


    def __W_to_dBm(self, value_in_w):
        '''
        Transform from W to dBm.
        '''
        dBm = 10 * np.log10(value_in_w / 0.001)
        return dBm


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


    def show_map(self):
        self.hardcode_obstacles()
        print(np.round(self.__raw_data))
        print(self.__raw_data[29, 0])


    def hardcode_obstacles(self):
        for y in range(3,9):
            self.__place_obstacle((11, y))
            self.__place_obstacle((12, y))
        
        lines = ((3,7), #13
                 (3,7),
                 (3,7),
                 (3,8),
                 (3,8),
                 (3,8),
                 (3,9),
                 (3,9), #20
                 (2,9),
                 (2,10),
                 (2,10),
                 (2,10),
                 (2,11),
                 (2,11),
                 (2,11),
                 (2,12),
                 (1,12),
                 (1,12),
                 (1,13))
        
        for line in lines:
            init, end = line
            for x in range(13, 30):
                for y in range(init, end + 1):
                    self.__raw_data[x, y] *= 1.015

        rows, cols = self.__raw_data.shape
        data = np.zeros((rows, cols))

        for x in range(rows):
            for y in range(cols):
                data[x, y] = self.__raw_data[x, y]
        return data
    

    def __place_obstacle(self, obs_coords):
        if isinstance(obs_coords, tuple):
            if isinstance(obs_coords[0], tuple):
                for obs_x, obs_y in obs_coords:
                    if (obs_x, obs_y) == self.__signal_origin:
                        print("Not possible to add obstacle in the signal origin pose...")
                    else:
                        self.__raw_data[obs_x, obs_y] = self.OBSTACLE_VALUE
            else:
                obs_x, obs_y = obs_coords
                if (obs_x, obs_y) == self.__signal_origin:
                    print("Not possible to add obstacle in the signal origin pose...")
                else:
                    self.__raw_data[obs_x, obs_y] = self.OBSTACLE_VALUE
        else:
            print("Wrong format, please introduce a single tuple coord (x,y) or a tuple of coords like ((x_a,y1),(x_b,y2), ...)")
