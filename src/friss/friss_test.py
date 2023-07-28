#! /usr/bin/env python
'''
This script is to test the friss module.
'''

import friss as fr

if __name__ == "__main__":
    heatmap = fr.Friss(world_sz=(30,30))
    heatmap.model_power_signal(origin=(5,3))
    heatmap.hardcode_obstacles()
