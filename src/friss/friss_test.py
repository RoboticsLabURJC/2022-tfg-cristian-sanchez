#! /usr/bin/env python
'''
This script is to test the friss module.
'''

import friss as fr


if __name__ == "__main__":
    heatmap = fr.Friss(world_sz=(5,5))
    heatmap.model_power_signal(origin=(1,2))
    heatmap.set_obstacle_effect()


# SIZE = 7

# def find_intersection(point1, point2, point3, point4, only_segments=False):
#     x1, y1 = point1
#     x2, y2 = point2
#     x3, y3 = point3
#     x4, y4 = point4

#     denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)

#     if denominator == 0:
#         return None
#     else:
#         x = int(((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denominator)
#         y = int(((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denominator)

#         if only_segments:
#             if (
#                 min(x1, x2) <= x <= max(x1, x2) and
#                 min(y1, y2) <= y <= max(y1, y2) and
#                 min(x3, x4) <= x <= max(x3, x4) and
#                 min(y3, y4) <= y <= max(y3, y4)
#             ):
#                 return x, y
#             else:
#                 return None
#         else:
#             return x, y
    

# def find_vertices(signal_point, obstacle_ends):
#     vertices = []
#     bounds = []

#     for vertex in obstacle_ends:        
#         vertices.append(vertex)
#         quad = get_quadrant(signal_point, vertex)
#         pointA, pointB, pointC, pointD = get_bounds(quad)
#         bounds.append((pointA, pointB, pointC, pointD))        
#         cand1 = find_intersection(signal_point, vertex, pointA, pointB)
#         cand2 = find_intersection(signal_point, vertex, pointC, pointD)
#         vertices.append(get_valid_points((cand1, cand2)))
    
#     if bounds[0] != bounds[1]:
#         vertices.append(get_corner(pointA, pointB, pointC, pointD, quad))

#     return set(vertices)


# def get_valid_points(points):
#     for point in points:
#         x, y = point
#         if 0 <= x < SIZE and 0 <= y < SIZE:
#             return point


# def get_corner(p1, p2, p3, p4, quadrant):
#     if not (p1 == p3 and p2 == p4):
#         if quadrant == 1:
#             return (0, SIZE - 1)
#         elif quadrant == 2:
#             return (0, 0)
#         elif quadrant == 3:
#             return (SIZE - 1, 0)
#         elif quadrant == 4:
#             return (SIZE - 1, SIZE - 1)
    

# def get_bounds(quadrant):
#     if quadrant == 1:
#         return (0,0), (0,1), (0, SIZE - 1), (SIZE - 1, SIZE - 1)
#     elif quadrant == 2:
#         return (0,0), (0,1), (0, 0), (1, 0)
#     elif quadrant == 3:
#         return (0,0), (0,1), (SIZE - 1, 0), (SIZE - 1, SIZE - 1)
#     elif quadrant == 4:
#         return (0, SIZE - 1), (SIZE - 1, SIZE - 1), (SIZE - 1, 0), (SIZE - 1, SIZE - 1)

# def get_quadrant(origin, dest):
#     ox, oy = origin
#     dx, dy = dest

#     diffX = dx - ox
#     diffY = dy - oy

#     if diffX <= 0 and diffY >= 0:
#         return 1
#     elif diffX <= 0 and diffY < 0:
#         return 2
#     elif diffX > 0 and diffY <= 0:
#         return 3
#     elif diffX > 0 and diffY > 0:
#         return 4
    

# def is_inside_polygon(polygon, point):
#     n_vertex = len(polygon)
#     x, _ = point
#     intersections = []

#     for i in range(n_vertex):
#         intersection = find_intersection(point, 
#                                          (x, SIZE - 1), 
#                                          polygon[i], 
#                                          polygon[(i + 1) % n_vertex], 
#                                          only_segments=True)
        
#         if intersection != None and intersection not in polygon and intersection != point:
#             intersections.append(intersection)

#     if len(intersections) % 2 != 0:
#         return True
    
#     return False


# if __name__ == "__main__":
#     poly = ((2,2), (2,4), (4,4), (4,2))
#     p = (3,3)
#     print(is_inside_polygon(poly, p))