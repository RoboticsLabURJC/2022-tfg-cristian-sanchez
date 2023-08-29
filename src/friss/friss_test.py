#! /usr/bin/env python
'''
This script is to test the friss module.
'''

import friss as fr

SZ = 5

class Point:
    def __init__(self, x, y):
        '''
        p = (x, y)
        '''
        self.x = x
        self.y = y

    def get_tuple(self):
        return (self.x, self.y)


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

                if self.x2 - self.x1 > 0:
                    self.direction = 'E'
                else:
                    self.direction = 'W'

            elif self.m > 0:
                if self.x2 - self.x1 > 0:
                    self.direction = 'NE'
                else:
                    self.direction = 'SW'
            
            else:
                if self.x2 - self.x1 > 0:
                    self.direction = 'SE'
                else:
                    self.direction = 'NW'


        except ZeroDivisionError:
            self.vertical = True
            self.m = None
            self.b = self.x1

            if self.y2 - self.y1 > 0:
                self.direction = 'N'
            else:
                self.direction = 'S'

        # self.show_parameters()


    def show_parameters(self):
        print((self.x1, self.y1))
        print((self.x2, self.y2))
        print(self.m)
        print(self.b)
        print(self.direction)
        print("horizontal:", self.horizontal)
        print("vertical:",self.vertical)
        print()
    

def intersection(line1, line2, finite=False):
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
            except TypeError:
                y = line2.m * x + line2.b

    if finite:
        inside_x_bounds = (line1.x1 <= x <= line1.x2 or line1.x2 <= x <= line1.x1) and (line2.x1 <= x <= line2.x2 or line2.x2 <= x <= line2.x1)
        inside_y_bounds = (line1.y1 <= y <= line1.y2 or line1.y2 <= y <= line1.y1) and (line2.y1 <= y <= line2.y2 or line2.y2 <= y <= line2.y1)
        # print(x, y)
        # print(line1.x1, line1.x2, line1.y1, line1.y2, line2.x1, line2.x2, line2.y1, line2.y2)
        # print(inside_x_bounds, inside_y_bounds)
        if inside_x_bounds and inside_y_bounds:
            return (x, y)
        else:
            return None

    return (x, y)

def in_map(map_sz, point):
    x, y = point
    return (0 <= x < map_sz) and (0 <= y < map_sz)


def in_direction(reference_p, intersection_p, direction):
    ref_x = reference_p.x
    ref_y = reference_p.y

    int_x, int_y = intersection_p

    if direction == 'N':
        return ref_y <= int_y
    elif direction == 'S':
        return ref_y >= int_y
    elif direction == 'E':
        return ref_x <= int_x
    elif direction == 'W':
        return ref_x >= int_x

    elif direction == 'NE':
        return ref_x <= int_x and ref_y <= int_y
    elif direction == 'NW':
        return ref_x >= int_x and ref_y <= int_y
    elif direction == 'SE':
        return ref_x <= int_x and ref_y >= int_y
    elif direction == 'SW':
        return ref_x >= int_x and ref_y >= int_y


def get_polygon_vertices(signal_origin, obstacle_vertices, edges):
    vertices = []
    vertices.append(obstacle_vertices[-1].get_tuple())
    vertices.append(obstacle_vertices[0].get_tuple())

    for obstacle_vertex in obstacle_vertices:
        l = Line(signal_origin, obstacle_vertex)

        for edge in edges:
            intersect_p = intersection(l, edge)

            if intersect_p != None and in_map(sz, intersect_p) and in_direction(origin, intersect_p, l.direction):
                vertices.append(intersect_p)

    if different_edges(vertices[-1], vertices[-2]):
        # Parallel edges 2 corners??
        corner = get_corner(vertices[-1], vertices[-2])
        if corner != None:
            vertices.insert(-1, corner)

    # Convert into valid coords (trunc)
    vert = [tuple(map(int, tup)) for tup in vertices]
    return vert

def different_edges(p1, p2):
    x1, y1 = p1
    x2, y2 = p2

    return x1 != x2 and y1 != y2


def get_corner(p1, p2, sz=SZ):
    x1, y1 = p1
    x2, y2 = p2

    if x1 == 0:
        if x2 == sz - 1:
            pass
        elif y2 == 0:
            return (0, 0)
        elif y2 == sz - 1:
            return (0, sz - 1)

    elif x1 == sz - 1:
        if x2 == 0:
            pass
        elif y2 == 0:
            return (sz - 1, 0)
        elif y2 == sz - 1:
            return (sz - 1, sz - 1)

    elif y1 == 0:
        if x2 == 0:
            return (0, 0)
        elif x2 == sz - 1:
            return (sz - 1, 0)
        elif y2 == sz - 1:
            pass

    elif y1 == sz - 1:
        if x2 == 0:
            return (0, sz - 1)
        elif x2 == sz - 1:
            return (sz - 1, sz - 1)
        elif y2 == 0:
            pass


def is_inside(point, polygon):
    _, y = point.get_tuple()
    h_line = Line(point, Point(sz - 1, y))
    intersections = set()
    for i in range(len(polygon)):
        x_o, y_o = polygon[i]
        try:
            x_f, y_f = polygon[i + 1]
        except IndexError:
            x_f, y_f = polygon[0]
    
        side = Line(Point(x_o, y_o), Point(x_f, y_f))

        intersect_p = intersection(h_line, side, finite=True)
        if intersect_p != None:
            intersections.add(intersect_p)

    print(intersections)
    return len(intersections) % 2 != 0


if __name__ == "__main__":
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
    obstacle_vertices = (Point(3,1), Point(1,3))

    poly = get_polygon_vertices(origin, obstacle_vertices, map_edges)
    
    points_inside = []
    for x in range(sz):
        for y in range(sz):
            if is_inside(Point(x, y), poly):
                points_inside.append((x,y))

    print(points_inside)
