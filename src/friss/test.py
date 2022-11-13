import numpy as np

def scale_val (val, old_range, new_range):
    old_min, old_max = old_range
    new_min, new_max = new_range

    norm = (val - old_min)/(old_max - old_min)
    new_val = (norm * (new_max - new_min)) + new_min

    return new_val

a = [1, 5, 3, 2]
b = [-30.0459970202808, -13.380855464062144, -11.422342079936467]

min = 1
max = 5
for elem in a:
    print(scale_val(elem, (min, max), (-1, 1)))