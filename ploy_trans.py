import numpy as np
import matplotlib.pyplot as plt
from typing import *


def plot_polygon(polygon: List[tuple], ax=None, **kwargs):
    '''
    绘制多边形,polygon: [(x1, y1), (x2,y2)..]
    '''
    if ax is None:
        ax = plt.gca()
    polygon.append(polygon[0])
    ax.plot([p[0] for p in polygon], [p[1] for p in polygon], **kwargs)
    return ax


def plot_line(lines: List[tuple], ax=None, **kwargs):
    '''
    绘制线段,line: ((x1, y1), (x2,y2))
    '''
    if ax is None:
        ax = plt.gca()
    for p in lines:
        ax.plot([p[0][0], p[1][0]], [p[0][1], p[1][1]], **kwargs)
    return ax


fig = None

data = []
with open("Robot/map.txt", "r") as f:
    data = f.readlines()

line_index = 0

t = int(data[line_index])
line_index += 1
for i in range(0, t):
    vertices_num = int(data[line_index])
    line_index += 1
    vertices = []
    while (vertices_num > 0):
        vertices.append((float(data[line_index]), float(data[line_index + 1])))
        line_index += 2
        vertices_num -= 1
    plot_polygon(vertices, fig, color='black')
    hole_num = int(data[line_index])
    line_index += 1
    while (hole_num > 0):
        hole_vertices_num = int(data[line_index])
        line_index += 1
        hole_vertices = []
        while (hole_vertices_num > 0):
            hole_vertices.append(
                (float(data[line_index]), float(data[line_index + 1])))
            line_index += 2
            hole_vertices_num -= 1
        plot_polygon(hole_vertices, fig, color='black')
        hole_num -= 1

line_num = int(data[line_index])
line_index += 1
lines = []
while (line_num > 0):
    lines.append(((float(data[line_index]), float(data[line_index + 1])),
                  (float(data[line_index + 2]), float(data[line_index + 3]))))
    line_index += 4
    line_num -= 1
plot_line(lines, fig, color='blue')

line_num = int(data[line_index])
line_index += 1
lines = []
while (line_num > 0):
    lines.append(((float(data[line_index]), float(data[line_index + 1])),
                  (float(data[line_index + 2]), float(data[line_index + 3]))))
    line_index += 4
    line_num -= 1
plot_line(lines, fig, color='red')

# point_num = int(data[line_index])
# line_index += 1
# points = []
# while (point_num > 0):
#     points.append((float(data[line_index]), float(data[line_index + 1])))
#     line_index += 2
#     point_num -= 1
# plt.plot([p[0] for p in points], [p[1] for p in points], color='green')


# plt.show()

data = "(24.25, 48.75)->(23.6464, 48.3536)->(23.5, 47.75)->(23.6464, 47.1464)->(24.75, 46.75)->(25.25, 45.25)->(26.75, 44.75)->(27.25, 43.25)->(28.8536, 42.3536)->(29, 41.75)->(28.8536, 41.1464)->(27.75, 40.75)->(25.25, 40.75)->(25.25, 39.25)->(24.8536, 39.1464)->(24.25, 39)->(20.25, 39)->(19.6464, 39.1464)->(19.5, 39.75)->(19.6464, 40.3536)->(20.8536, 41.1464)->(21, 41.75)->(21, 45.75)->(20.8536, 46.3536)->(19.75, 46.75)->(18.8536, 48.3536)->(18.25, 48.5)->(16.25, 48.5)->(15.6464, 48.3536)->(15.5, 47.75)->(15.5, 45.75)->(15.6464, 45.1464)->(16.25, 45)->(16.8536, 45.1464)->(17.25, 44.75)"

data = data.split("->")


data = [eval(data[i]) for i in range(0, len(data))]

# plot data
plt.plot([p[0] for p in data], [p[1] for p in data], color='green')
plt.scatter([p[0] for p in data], [p[1] for p in data], color='red')
plt.show()
