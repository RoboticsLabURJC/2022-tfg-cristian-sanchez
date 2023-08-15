#! /usr/bin/env python
'''
This script is to test the friss module.
'''

import friss as fr

class Point:
    def __init__(self, x, y):
        '''
        p = (x, y)
        '''
        self.x = x
        self.y = y

class Line:
    def __init__(self, point1, point2):
        '''
        y = mx + b
        '''
        self.x1 = point1.x
        self.y1 = point1.y
        self.x2 = point2.x
        self.y2 = point2.y

        try:
            self.m = int((self.y2 - self.y1) / (self.x2 - self.x1))
            self.b = self.y1 - self.m * self.x1
        except ZeroDivisionError:
            self.m = None
            self.b = self.x1
    

def intersection(line1, line2):
    try:
        try:
            x = (line2.b - line1.b) / (line1.m - line2.m)
        except ZeroDivisionError:
            return None
        
        y = line1.m * x + line1.b
    except TypeError:
        if line1.m == None:
            x = line1.b
            y = line2.b
        else:
            x = line2.b
            y = line1.b

    return (x, y)

if __name__ == "__main__":
    # heatmap = fr.Friss(world_sz=(30,30))
    # heatmap.model_power_signal(origin=(5,3))
    # heatmap.hardcode_obstacles()

    # heatmap = fr.Friss(world_sz=(5,5))
    # heatmap.model_power_signal(origin=(1,2))
    # heatmap.p_obs(4,1)
    # heatmap.shoow()

    sz = 5

    corner_left_up = Point(0, 0)
    corner_left_down = Point(sz - 1, 0)
    corner_right_up = Point(0, sz - 1)
    corner_right_down = Point(sz - 1, sz - 1)

    edge_top = Line(corner_left_up, corner_right_up)
    edge_left = Line(corner_left_up, corner_left_down)
    edge_bot = Line(corner_left_down, corner_right_down)
    edge_right = Line(corner_right_up, corner_right_down)

    print(intersection(edge_top, edge_bot))
