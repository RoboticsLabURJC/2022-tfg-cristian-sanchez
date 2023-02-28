[![linting: pylint](https://img.shields.io/badge/linting-pylint-yellowgreen)](https://github.com/PyCQA/pylint)

<p align="center">
<img src="https://github.com/RoboticsLabURJC/2022-tfg-cristian-sanchez/blob/main/docs/images/friss_app.gif" alt="Friss demo" width="600"/>
</p>

## Introduction
This app let us to generate the RF signal data and modify every parameter in the [friss equation](https://www.gaussianwaves.com/2013/09/friss-free-space-propagation-model/):

<p align="center">
<img src="https://github.com/RoboticsLabURJC/2022-tfg-cristian-sanchez/blob/main/docs/images/Friss_formula.png" alt="Friss formula" width="200"/>
</p>

You will see two displays:

1. **Default**, where we can see how the values change by looking through the colorbar. This is because the graphic representation, automatically adjust the upper and lower limit, so the display doesn't change at all.

2. **Fixed**, which fix the auto adjustment so we can intuitively see changes graphically. The reason behind is the same explained previously, in this case, we stablish max and min value for the colorbar (that must be inside the bounds to see the effect) , to representate changing values. The result is a changing color image where we can see the effect on a better way.

## Resolution and size
These parameters are important:

1. **MapResolution**, defines the size of the cell displayed. It means that if resolution is equal to 2, for example, cells are twice bigger.

2. **MapSize**, determine the number of cells displayed, usually calculated in meters.

Summarizing, if you have Mapsize equal to 80 and MapResolution equal to 0'5 means that you will see a world of 80 x 80 units, with cells of 0'5 x 0'5 units.

## Usage guide

To use it, follow these simple steps after clonning this repository:
```
cd 2022-tfg-cristian-sanchez/src/friss
python3 main.py
```

To exit, just press q. Enjoy!
