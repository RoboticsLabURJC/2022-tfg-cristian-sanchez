---
title: "Next weeks. Friss model module."
last_modified_at: 2023-01-25T19:43:00
categories:
  - Blog
tags:
  - Friss
  - Python
---
The following weeks, I was working on developing an app to model a radio frequency signal.

To do it, I used the [Friss propagation model](https://www.gaussianwaves.com/2013/09/friss-free-space-propagation-model/), which gives us the received power of a signal in Watts.

<p align="center">
<img src="/2022-tfg-cristian-sanchez/images/Friss_formula.png" alt="Friss formula" width="500"/>
</p>

This formula also allows us to model different spaces by changing the "n" exponent.

<p align="center">
<img src="/2022-tfg-cristian-sanchez/images/Friss_n_exponent_table.png" alt="n exponent table" width="500"/>
</p>

## First approach

So the first thing I tried is to simply implementing the formula numerically, and test it to see how it works.

```Python3.8
P_t = 10    # Trasmitter power (Watts)
G_t = 1     # Gain of transmitter (Watts)
G_r = 1     # Gain of receiver (Watts)
lmd = 1     # Wavelength (250/260 MHz)
L = 1       # Other losses factors L = 1 --> no losses
n = 2       # Path Loss Exponent (for different environments)

d = 1

P_r = P_t * (G_t * G_r * (lmd)**2)/(((4 * np.pi)**2) * (d**n) * L)
```

But wasn't enough to see it's effect, so I used a numpy matrix simulating different distances between cells. And here is what I got:

<p align="center">
<img src="/2022-tfg-cristian-sanchez/images/Friss_first_results.png" alt="First results" width="500"/>
</p>

## Making the module

I decided to make a separated module (without using ROS), to create an app that permits, whoever, to modify and see changes dinamically by using the Friss model.

So I created a Friss class that haves this API:
