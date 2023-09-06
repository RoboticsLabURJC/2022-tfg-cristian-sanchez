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


def get_polygon_vertices(signal_origin, obstacle_vertices, edges, rounded=False):
    vertices = []
    vertices.append(obstacle_vertices[-1].get_tuple())
    vertices.append(obstacle_vertices[0].get_tuple())

    for obstacle_vertex in obstacle_vertices:
        l = Line(signal_origin, obstacle_vertex)

        for edge in edges:
            intersect_p = intersection(l, edge)

            if intersect_p != None and in_map(sz, intersect_p) and in_direction(origin, intersect_p, l.direction):
                vertices.append(intersect_p)

    print(check_edges(vertices[-1], vertices[-2]))
    result = check_edges(vertices[-1], vertices[-2])
    if result == 'different edge':
        corner = get_corner(vertices[-1], vertices[-2])
        if corner != None:
            vertices.insert(-1, corner)
    elif result == 'opposite edge':
        x1, y1 = vertices[-1]
        x2, y2 = vertices[-2]
        a = Line(signal_origin, Point(x1, y1))
        b = Line(signal_origin, Point(x2, y2))

        vertices.insert(-1, get_corner_dir(a.direction))
        vertices.insert(-1, get_corner_dir(b.direction))
        print(a.direction)
        print(b.direction)

    # if different_edges(vertices[-1], vertices[-2]):
    #     # Parallel edges 2 corners??
    #     corner = get_corner(vertices[-1], vertices[-2])
    #     if corner != None:
    #         vertices.insert(-1, corner)

    # Convert into valid coords (trunc)
    if rounded:
        vert = [tuple(map(int, tup)) for tup in vertices]
        return vert
    else:
        return vertices

def different_edges(p1, p2):
    x1, y1 = p1
    x2, y2 = p2

    return x1 != x2 and y1 != y2

def check_edges(p1, p2):
    x1, y1 = p1
    x2, y2 = p2

    if x1 == 0 or x1 == sz - 1:
        if x2 == x1:
            return "same edge"
        elif (x1 == 0 and x2 == sz - 1) or (x1 == sz - 1 and x2 == 0):
            return "opposite edge"

    if y1 == 0 or y1 == sz - 1:
        if y2 == y1:
            return "same edge"
        elif (y1 == 0 and y2 == sz - 1) or (y1 == sz - 1 and y2 == 0):
            return "opposite edge"
        
    return "different edge"

def get_corner_dir(dir):
    if dir == 'NW':
        return (0, sz - 1)
    elif dir == 'SW':
        return (0, 0)
    elif dir == 'SE':
        return (sz - 1, 0)
    elif dir == 'NE':
        return (sz - 1, sz - 1)
    else:
        return -1


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


def is_inside(point, polygon, edge_points):
    
    x, y = point.get_tuple()
    if (x, y) not in edge_points:
        h_line = Line(point, Point(sz - 1, y))
    else:
        return False
    
    intersections = set()

    
    for i in range(len(polygon)):
        x_o, y_o = polygon[i]
        try:
            x_f, y_f = polygon[i + 1]
        except IndexError:
            x_f, y_f = polygon[0]
    
        side = Line(Point(x_o, y_o), Point(x_f, y_f))

        intersect_p = intersection(h_line, side, finite=True)
        # print(intersect_p)
        if intersect_p != None:
            intersections.add(intersect_p)

    # print("for point:", (x,y), "intersects ", len(intersections), "in", intersections)
    # print()

    return len(intersections) % 2 != 0


def get_edge_points(poly):
    

    poly_coords = []
    for i in range(len(poly)):
        x_po, y_po = poly[i]
        try:
            x_pf, y_pf = poly[i + 1]
        except IndexError:
            x_pf, y_pf = poly[0]

        side = Line(Point(x_po, y_po), Point(x_pf, y_pf))

        for x in range(int(min(x_po, x_pf)), int(max(x_po, x_pf) + 1)):
            for y in range(int(min(y_po, y_pf)), int(max(y_po, y_pf) + 1)):
                try:
                    if y == side.m * x + side.b:
                        poly_coords.append((x, y))
                except TypeError:
                    if x == side.b and min(y_po, y_pf) <= y <= max(y_po, y_pf):
                        poly_coords.append((x, y))

    

    return list(set(poly_coords))

def get_non_obstacle_points(edge_points, obstacle_vertices):
    obstacle = Line(obstacle_vertices[0], obstacle_vertices[1])
    x_o, y_o = obstacle_vertices[0].get_tuple()
    x_f, y_f = obstacle_vertices[1].get_tuple()

    obstacle_coords = []
    for x in range(int(min(x_o, x_f)), int(max(x_o, x_f) + 1)):
        for y in range(int(min(y_o, y_f)), int(max(y_o, y_f) + 1)):
            try:
                if y == obstacle.m * x + obstacle.b:
                    obstacle_coords.append((x, y))
            except TypeError:
                if x == obstacle.b and min(y_o, y_f) <= y <= max(y_o, y_f):
                    obstacle_coords.append((x, y))

    for coord in obstacle_coords:
        edge_points.remove(coord)

    return edge_points

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
    obstacle_vertices = (Point(3,1), Point(0,3))

    poly = get_polygon_vertices(origin, obstacle_vertices, map_edges)
    print(poly)
    
    edge_points = get_edge_points(poly)
    points_inside = []
    for x in range(sz):
        for y in range(sz):
            if is_inside(Point(x, y), poly, edge_points):
                points_inside.append((x,y))

    result = get_non_obstacle_points(edge_points, obstacle_vertices)
    result.extend(points_inside)
    print(result)
