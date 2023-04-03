'''
绘制[((x1, y1),(x2, y2), (x3, y3)),...]格式的三角面片
'''
import numpy as np
import matplotlib.pyplot as plt
from typing import *


def plot_triangles(triangles, ax=None, **kwargs):
    '''
    绘制三角面片,填充颜色,triangles: [((x1, y1),(x2, y2), (x3, y3)),...]
    '''
    if ax is None:
        ax = plt.gca()
    for triangle in triangles:
        ax.fill([p[0] for p in triangle], [p[1] for p in triangle], **kwargs)
    return ax


def plot_polygon(polygon: List[tuple], ax=None, **kwargs):
    '''
    绘制多边形,polygon: [(x1, y1), (x2,y2)..]
    '''
    if ax is None:
        ax = plt.gca()
    polygon.append(polygon[0])
    ax.plot([p[0] for p in polygon], [p[1] for p in polygon], **kwargs)
    return ax


# trangles = [((50, 50), (0, 0), (50, 100)), ((50, 50), (100, 0), (50, 100))]
# ax = plot_triangles(trangles)
# plt.show()

# 画布区域(0,0)->(50, 50)

# for i in range(1, len(points)+1):
#     plt.xlim(-1, 51)
#     plt.ylim(-1, 51)
#     plt.plot([p[0] for p in points[0:i]], [p[1] for p in points[0:i]])
#     plt.show()
#     if i == len(points):
#         plt.title('polygon')
triangles = [((0, 0),(0, 27),(1.5, 27)),((1.5, 27.5),(0.5, 27.5),(0.5, 34.5)),((1.5, 27.5),(0.5, 34.5),(2.5, 34.5)),((1.5, 27.5),(2.5, 34.5),(2.5, 34)),((1.5, 27.5),(2.5, 34),(3, 34)),((3, 35),(0, 35),(0, 50)),((3, 35),(0, 50),(50, 50)),((3, 35),(50, 50),(50, 35)),((3, 35),(50, 35),(47, 35)),((3, 35),(47, 35),(47, 34)),((47.5, 34),(47.5, 34.5),(49.5, 34.5)),((47.5, 34),(49.5, 34.5),(49.5, 27.5)),((47.5, 34),(49.5, 27.5),(48, 27.5)),((47.5, 34),(48, 27.5),(48, 27)),((48, 27),(50, 27),(50, 0)),((48, 27),(50, 0),(0, 0)),((48, 27),(0, 0),(1.5, 27)),((48, 27),(1.5, 27),(1.5, 27.5)),((48, 27),(1.5, 27.5),(3, 34)),((48, 27),(3, 34),(3, 35)),((48, 27),(3, 35),(47, 34)),((48, 27),(47, 34),(47.5, 34)),]
points = [
(0,27),(1.5,27),(1.5,27.5),(0.5,27.5),(0.5,34.5),(2.5,34.5),(2.5,34),(3,34),(3,35),(0,35),(0,50),(50,50),(50,35),(47,35),(47,34),(47.5,34),(47.5,34.5),(49.5,34.5),(49.5,27.5),(48,27.5),(48,27),(50,27),(50,0),(0,0),]

ax=plot_polygon(points)
plot_triangles(triangles, ax=ax)
plt.show()