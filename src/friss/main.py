#! /usr/bin/env python
import numpy as np
import matplotlib.pylab as plt
        
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

        # norm_data = data / np.linalg.norm(data)
        # self.__show_data(norm_data)
        self.__show_data(data)

    def model_signal_losses(self, rows, cols, origin=(0,0)):
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
        power_recv = self.__friss_formula(d_ref) + 20 * np.log10(d_ref/d_final)
        return power_recv
                 
    def __show_data(self, data):
        plt.pcolormesh(data, cmap = 'winter')
        plt.title('Heatmap')
        plt.show()

    def test(self, dist, new_dist):
        print("Pt (", dist, "m ):", self.__W_to_dBm(self.__power_t), "dBm")
        print("Pt (", dist, "m ):", self.__W_to_dB(self.__power_t), "dB")
        print("Pr (", dist, "m ):", self.__friss_formula(dist), "dBm")
        print("Pr (", new_dist, "m ), without friss:", self.__rcv_power_with_ref(dist, new_dist), "dBm")
        print("Pr (", new_dist, "m ), with friss:", self.__friss_formula(new_dist), "dBm")



if __name__ ==  "__main__":
    my_model = Friss(power_tras=50.0, gain_tras=1, gain_recv=2)
    my_model.test(100, 10 * (10**3))
    print(" --------- ")
    my_model.test(10, 100)
    my_model.model_power_signal(100, 100, (20, 20))
    # my_model.model_signal_losses(25, 25, (20, 20))