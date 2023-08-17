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

        self.vertical = False
        self.horizontal = False

        try:
            self.m = (self.y2 - self.y1) / (self.x2 - self.x1)
            self.b = self.y1 - self.m * self.x1

            if self.m == 0:
                self.horizontal = True

        except ZeroDivisionError:
            self.vertical = True
            self.b = self.x1
    

def intersection(line1, line2):
    x, y = None, None

    if (line1.horizontal and line2.horizontal) or (line1.vertical and line2.vertical):
        return None
    
    else:
        if line1.horizontal:
            y = line1.b
        elif line1.vertical:
            x = line1.b
        
        if line2.horizontal:
            y = line2.b
        elif line2.vertical:
            x = line2.b

        if x == None:
            x = (line2.b - line1.b) / (line1.m - line2.m)

        if y == None:
            try:
                y = line1.m * x + line1.b
            except AttributeError:
                y = line2.m * x + line2.b     

    return (x, y)

def in_map(map_sz, point):
    x, y = point
    return (0 <= x < map_sz) and (0 <= y < map_sz)

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

    map_edges = (edge_top, edge_left, edge_bot, edge_right)    

    origin = Point(1,2)
    obstacle_vertex = Point(3,1)
    obs_line = Line(origin, obstacle_vertex)

    for edge in map_edges:
        intersect_p = intersection(obs_line, edge)

        if intersect_p != None and in_map(sz, intersect_p):
            print(intersection(obs_line, edge))