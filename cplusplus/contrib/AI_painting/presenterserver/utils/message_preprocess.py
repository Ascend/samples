import numpy as np
import os
import json

object_to_idx = {"sky": 1, "sand": 2, "sea": 3, "mountain": 4, "rock": 5,
                 "earth": 6, "tree": 7, "water": 8, "land": 9, "grass": 10,
                 "path": 11, "dirt": 12, "river": 13, "hill": 14, "field": 15,
                 "lake": 16, "__image__": 0}

layout = {}
keys = ['valid', 'coor_x', 'coor_y', 'size']
values = [1, 128, 128, 100]  # coord_x ,coord_y [0, 255]
sky = dict(zip(keys, values))
layout.setdefault('sky', []).append(sky)

keys = ['valid', 'coor_x', 'coor_y', 'size']
values = [0, 0, 0, 0]
earth = dict(zip(keys, values))
layout.setdefault('earth', []).append(earth)

keys = ['valid', 'coor_x', 'coor_y', 'size']
values = [1, 50, 80, 30]
grass = dict(zip(keys, values))
layout.setdefault('grass', []).append(grass)

print('\nlayout:', layout)
print('\nsky:', layout['sky'][0])
print('\nsky size:', layout['sky'][0].get('size'))
print('\nearth:', layout['earth'][0])
print('\ngrass:', layout['grass'][0])
