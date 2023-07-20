#! /usr/bin/env python
'''
FRISS

With this module, you are able to simulate a radio frequency
signal, by using the friss theory.
'''
import numpy as np

# Common frequency profiles
FREQ_WIFI = 2.4 * (10**9)
FREQ_5G = 5 * (10**9)
FREQ_FM = 100 * (10**6)

class Friss:
    '''
    Here is a summary of the API:

        reset_world         --> Resets current world.
        get_world_sz        --> Returns world size.
        set_values          --> Set friss formula parameters.
        model_power_signal  --> Fills power data.
        model_signal_losses --> Fills losses data.
        print_all           --> Prints all parameters.
        test                --> Model some cases.
    '''
    C = 3.0 * (10 ** 8) # Light speed

    def __init__(self, power_tras=1.0, gain_tras=1.0, gain_recv=1.0,
                 freq=FREQ_WIFI, losses_factor=2.4, losses_path=2.0,
                 world_sz=(50, 50), resolution=1.0):
        '''
        Constructor. Here it's filled all parameters to create a Friss model.
        '''

        self.__power_t = power_tras                     # Pt
        self.__gain_t = gain_tras                       # Gt
        self.__gain_r = gain_recv                       # Gr
        self.__lambda = self.__get_lambda(freq)         # Lambda
        self.__losses_f = losses_factor                 # L
        self.__losses_p = losses_path                   # n

        self.__resolution = resolution # units per cell, 0.5 =cell side is 0.5 m
        self.__world_sz = world_sz # defines ticks in x, y (20, 20) = 20x20 map
        self.__raw_data = self.__generate_data_grid() # data grid
        self.__signal_origin = (0,0)


    def __generate_data_grid(self):
        '''
        Creates a grid with empty data to fill.
        '''
        l_magnitude, _ = self.__world_sz
        l_grid = int(l_magnitude / self.__resolution)

        return np.empty((l_grid, l_grid))

    def reset_world(self, resolution, size):
        '''
        Resets current world to create a new one with new resolution and/or size.
        '''
        self.__resolution = resolution
        self.__world_sz = size
        self.__raw_data = self.__generate_data_grid()


    def get_world_sz(self):
        '''
        Returns world size.
        '''
        return self.__world_sz


    def set_values(self, power_tras, gain_tras, gain_recv, freq, losses_factor, losses_path):
        '''
        Allows the modification of the friss formula parameters.
        '''
        self.__power_t = power_tras                     # Pt
        self.__gain_t = gain_tras                       # Gt
        self.__gain_r = gain_recv                       # Gr
        self.__lambda = self.__get_lambda(freq)         # Lambda
        self.__losses_f = losses_factor                 # L
        self.__losses_p = losses_path                   # n


    def __get_lambda(self, freq):
        '''
        Returns lambda.
        '''
        return self.C/freq


    def __friss_formula(self, dist):
        '''
        Returns received power in a certain distances in dBm.
        '''
        if dist == 0:
            dist = 0.9 * self.__resolution

        power_recv = self.__power_t * (self.__gain_t * self.__gain_r * (self.__lambda)**2)/(((4 * np.pi)**2) * (dist**self.__losses_p) * self.__losses_f)
        power_recv_dBm = self.__W_to_dBm(power_recv)
        return power_recv_dBm


    def __propagation_path_loss(self, dist):
        '''
        Returns loss in a certain distances in dB.
        '''
        if dist == 0:
            dist = 0.9 * self.__resolution

        # Empty space case
        loss = -10 * np.log10((self.__lambda**2)/((4 * np.pi * dist)**2))
        return loss


    def model_power_signal(self, origin=(0,0)):
        '''
        Fills data using friss formula and returns it.
        '''
        x_origin, y_origin = origin
        self.__signal_origin = origin
        rows, cols = self.__raw_data.shape
        data = np.zeros((rows, cols))

        for x in range(rows):
            for y in range(cols):
                dist_to_origin = (((x - x_origin)**2 + (y - y_origin)**2) ** (1/2)) * self.__resolution
                pwr_recv = self.__friss_formula(dist_to_origin)
                data[x, y] = pwr_recv

                self.__raw_data[x, y] = pwr_recv # For debugging

        return data


    def model_signal_losses(self, origin=(0,0)):
        '''
        Fills data using propagation path loss formula and returns it.
        '''
        x_origin, y_origin = origin
        rows, cols = self.__raw_data.shape
        data = np.zeros((rows, cols))
        for x in range(rows):
            for y in range(cols):
                dist_to_origin = (((x - x_origin)**2 + (y - y_origin)**2) ** (1/2)) * self.__resolution
                losses = self.__propagation_path_loss(dist_to_origin)
                data[x, y] = losses

                self.__raw_data[x, y] = losses # For debuggin

        return data


    def __W_to_dBm(self, value_in_w):
        '''
        Transform from W to dBm.
        '''
        dBm = 10 * np.log10(value_in_w / 0.001)
        return dBm


    # -- DEBUG -- #
    def print_all(self):
        '''
        Shows every friss parameter used in the formulas.
        '''
        print("****************")
        print("Current Friss values:")
        print("\tPt:",self.__power_t, "W")
        print("\tGt:",self.__gain_t, "W")
        print("\tGr:",self.__gain_r, "W")
        print("\tLm:",self.__lambda)
        print("\tl:",self.__losses_f)
        print("\tn:",self.__losses_p)
        print("****************")


    def test(self, dist, new_dist):
        '''
        Function used for testing purposes.
        '''
        self.print_all()
        print("P transmiter (", dist, "m ):", self.__W_to_dBm(self.__power_t), "dBm")
        print("P receiver (", dist, "m ):", self.__friss_formula(dist), "dBm")
        print("P transmiter (", new_dist, "m ):", self.__W_to_dBm(self.__power_t), "dBm")
        print("P receiver (", new_dist, "m ):", self.__friss_formula(new_dist), "dBm")

        current_range = (np.min(self.__raw_data), np.max(self.__raw_data))
        Pr = self.__friss_formula(new_dist)
        print("\nFor", new_dist,"m in dBm:")
        print("\tPr:", Pr,"in range", current_range)
        print("\n")


    def show_map(self):
        # self.__place_obstacle(((3,3), (2,2)))
        self.hardcode_obstacles()
        print(np.round(self.__raw_data))
        print(self.__raw_data[29, 0])

    def hardcode_obstacles(self):
        for y in range(5,11):
            self.__place_obstacle((20, y))
            self.__place_obstacle((21, y))

        lines = ((5,10), 
                 (5,10),
                 (5,11),
                 (6,11),
                 (6,12),
                 (6,12))
        
        for line in lines:
            init, end = line
            for x in range(22, 30):
                for y in range(init, end + 1):
                    self.__raw_data[x, y] *= 1.1

        rows, cols = self.__raw_data.shape
        data = np.zeros((rows, cols))

        for x in range(rows):
            for y in range(cols):
                data[x, y] = self.__raw_data[x, y]
        return data









































    # EXPERIMENTAL STUFF...
    def __place_obstacle(self, obs_coords):
        if isinstance(obs_coords, tuple):
            if isinstance(obs_coords[0], tuple):
                for obs_x, obs_y in obs_coords:
                    if (obs_x, obs_y) == self.__signal_origin:
                        print("Not possible to add obstacle in the signal origin pose...")
                    else:
                        self.__raw_data[obs_x, obs_y] = -99
            else:
                obs_x, obs_y = obs_coords
                if (obs_x, obs_y) == self.__signal_origin:
                    print("Not possible to add obstacle in the signal origin pose...")
                else:
                    self.__raw_data[obs_x, obs_y] = -99
        else:
            print("Wrong format, please introduce a single tuple coord (x,y) or a tuple of coords like ((x_a,y1),(x_b,y2), ...)")


    def __find_intersection(self, point1, point2, point3, point4, only_segments=False):
        x1, y1 = point1
        x2, y2 = point2
        x3, y3 = point3
        x4, y4 = point4

        denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)

        if denominator == 0:
            return None
        else:
            x = int(((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denominator)
            y = int(((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denominator)

            if only_segments:
                if (
                    min(x1, x2) <= x <= max(x1, x2) and
                    min(y1, y2) <= y <= max(y1, y2) and
                    min(x3, x4) <= x <= max(x3, x4) and
                    min(y3, y4) <= y <= max(y3, y4)
                ):
                    return x, y
                else:
                    return None
            else:
                return x, y
        

    def __find_vertices(self, signal_point, obstacle_ends):
        vertices = list(obstacle_ends)
        bounds = []

        left = 0
        right = 0
        down = 0
        up =  0

        for vertex in obstacle_ends:        
            quad = self.__get_quadrant(signal_point, vertex)
            pointA, pointB, pointC, pointD = self.__get_bounds(quad)
            bounds.append((pointA, pointB, pointC, pointD))
            cand1 = self.__find_intersection(signal_point, vertex, pointA, pointB)
            cand2 = self.__find_intersection(signal_point, vertex, pointC, pointD)
            valid_point = self.__get_valid_points((cand1, cand2))

            if valid_point[0] == 0:
                up += 1
            if valid_point[0] == self.__world_sz[0] - 1:
                down += 1
            if valid_point[1] == 0:
                left += 1
            if valid_point[1] == self.__world_sz[0] - 1:
                right += 1

            if valid_point not in vertices:
                vertices.append(valid_point)

            print(vertex, quad, bounds, cand1, cand2, vertices)

        print(up, down, left, right)

        if up > 0 and left > 0:
            vertices.append((0, 0))
        if up > 0 and right > 0:
            vertices.append((0, self.__world_sz[0] - 1))
        if down > 0 and left > 0:
            vertices.append((self.__world_sz[0] - 1, 0))
        if down > 0 and right > 0:
            vertices.append((self.__world_sz[0] - 1, self.__world_sz[0] - 1))

        if len(vertices) == 5:
            last_position = vertices[-1]
            previous_position = vertices[-2]
            vertices[-1] = previous_position
            vertices[-2] = last_position

        first_coll = vertices.pop(2)
        vertices.insert(0, first_coll)

        return tuple(vertices)


    def __get_valid_points(self, points):
        for point in points:
            if point == None:
                continue

            x, y = point
            if 0 <= x < self.__world_sz[0] and 0 <= y < self.__world_sz[0]:
                return point


    def __get_corner(self, p1, p2, p3, p4, quadrant):
        if not (p1 == p3 and p2 == p4):
            if quadrant == 1:
                return (0, self.__world_sz[0] - 1)
            elif quadrant == 2:
                return (0, 0)
            elif quadrant == 3:
                return (self.__world_sz[0] - 1, 0)
            elif quadrant == 4:
                return (self.__world_sz[0] - 1, self.__world_sz[0] - 1)
        

    def __get_bounds(self, quadrant):
        if quadrant == 1:
            return (0,0), (0,1), (0, self.__world_sz[0] - 1), (self.__world_sz[0] - 1, self.__world_sz[0] - 1)
        elif quadrant == 2:
            return (0,0), (0,1), (0, 0), (1, 0)
        elif quadrant == 3:
            return (0,0), (1,0), (self.__world_sz[0] - 1, 0), (self.__world_sz[0] - 1, self.__world_sz[0] - 1)
        elif quadrant == 4:
            return (0, self.__world_sz[0] - 1), (self.__world_sz[0] - 1, self.__world_sz[0] - 1), (self.__world_sz[0] - 1, 0), (self.__world_sz[0] - 1, self.__world_sz[0] - 1)

    def __get_quadrant(self, origin, dest):
        ox, oy = origin
        dx, dy = dest

        diffX = dx - ox
        diffY = dy - oy

        if diffX <= 0 and diffY >= 0:
            return 1
        elif diffX <= 0 and diffY < 0:
            return 2
        elif diffX > 0 and diffY <= 0:
            return 3
        elif diffX > 0 and diffY > 0:
            return 4
        

    def __is_inside_polygon(self, polygon, point):
        n_vertex = len(polygon)
        x, _ = point
        intersections = []

        for i in range(n_vertex):
            intersection = self.__find_intersection(point, 
                                                    (x, self.__world_sz[0] - 1), 
                                                    polygon[i], 
                                                    polygon[(i + 1) % n_vertex], 
                                                    only_segments=True)
            
            # print(point, (x, self.__world_sz[0] - 1), polygon[i], polygon[(i + 1) % n_vertex], intersection)

            if intersection != None and intersection not in polygon and point not in polygon:
                intersections.append(intersection)
        # print(intersections)
        if len(intersections) % 2 != 0:
            return True
        
        return False
    
    def set_obstacle_effect(self):
        obstacles = ((2,1), (2,2), (2,3))
        self.__place_obstacle(obstacles)

        obs_init = obstacles[0]
        obs_end = obstacles[-1]

        poly = self.__find_vertices(self.__signal_origin, (obs_init, obs_end))

        print(np.round(self.__raw_data))

        rows, cols = self.__raw_data.shape
        for x in range(rows):
            for y in range(cols):
                point = (x, y)
                if self.__is_inside_polygon(poly, point):
                    # print("AAAAAAAAAAAAAA")
                    # print((x,y))
                    self.__raw_data[x, y] *= 0.75

        print(np.round(self.__raw_data))
        print(poly)

    def __del__(self):
        pass
