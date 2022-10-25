#! /usr/bin/env python
import numpy as np

# -- RECEIVED POWER -- #
P_t = 10    # Trasmitter power (Watts)
G_t = 1     # Gain of transmitter (Watts)
G_r = 1     # Gain of receiver (Watts)
lmd = 1     # Wavelength
L = 1       # Other losses factors L = 1 --> no losses
n = 2       # Path Loss Exponent (for different environments)

d = 1       # Distance (m)

# Friss formula
P_r = P_t * (G_t * G_r * (lmd)**2)/(((4 * np.pi)**2) * (d**n) * L)

# -- PROPAGATION PATH LOSS -- #
# Empty space
P_l = -10 * np.log10((lmd**2)/((4 * np.pi * d)**2))