#! /usr/bin/env python
import numpy as np

# -- RECEIVED POWER -- #
P_t = 10    # Trasmitter power (Watts)
G_t = 1     # Gain of transmitter (Watts)
G_r = 1     # Gain of receiver (Watts)
lmd = 1     # Wavelength (250/260 MHz)
L = 1       # Other losses factors L = 1 --> no losses
n = 2       # Path Loss Exponent (for different environments)

d = 1       # Distance (m)

# Friss formula
P_r = P_t * (G_t * G_r * (lmd)**2)/(((4 * np.pi)**2) * (d**n) * L)

# -- PROPAGATION PATH LOSS -- #
# Empty space
P_l = -10 * np.log10((lmd**2)/((4 * np.pi * d)**2))

grid = np.zeros((9, 9))

x_origin, y_origin = (5, 5)
x_l, y_l = grid.shape
for x in range(x_l):
    for y in range(y_l):
        d = ((x - x_origin)**2 + (y - y_origin)**2)**(1/2)
        if d == 0:
            # grid[x, y] = 0.0
            grid[x, y] = 1.0
        else:
            # grid[x, y] = -10 * np.log10((lmd**2)/((4 * np.pi * d)**2))        
            grid[x, y] = P_t * (G_t * G_r * (lmd)**2)/(((4 * np.pi)**2) * (d**n) * L)

grid = np.round(grid, 2)
print(grid)