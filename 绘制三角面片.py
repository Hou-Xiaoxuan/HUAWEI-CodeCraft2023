'''
绘制[((x1, y1),(x2, y2), (x3, y3)),...]格式的三角面片
'''
import numpy as np
import matplotlib.pyplot as plt
from typing import *


def plot_triangles(triangles, ax=None, **kwargs):
    '''
    绘制三角面片
    '''
    if ax is None:
        ax = plt.gca()
    for triangle in triangles:
        ax.plot([triangle[0][0], triangle[1][0], triangle[2][0], triangle[0][0]],
                [triangle[0][1], triangle[1][1], triangle[2][1], triangle[0][1]], **kwargs)
    return ax

# trangles = [((50, 50), (0, 0), (50, 100)), ((50, 50), (100, 0), (50, 100))]
# ax = plot_triangles(trangles)
# plt.show()
