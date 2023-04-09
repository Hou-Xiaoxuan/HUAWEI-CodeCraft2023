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


def plot_grid(ax=None, **kwargs):
    if ax is None:
        ax = plt.gca()
    # 绘制0.5 * 0. 5的网格 宽100 长100 不要横纵坐标
    ax.set_xticks(np.arange(0, 100, 0.5))
    ax.set_yticks(np.arange(0, 100, 0.5))
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    ax.grid(True, **kwargs)
    return ax


fig = None

fig = plot_grid(fig)

data = []
with open("./Robot/map.txt", "r") as f:
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

line_num = int(data[line_index])
line_index += 1
lines = []
while (line_num > 0):
    lines.append(((float(data[line_index]), float(data[line_index + 1])),
                  (float(data[line_index + 2]), float(data[line_index + 3]))))
    line_index += 4
    line_num -= 1
plot_line(lines, fig, color='green')


# point_num = int(data[line_index])
# line_index += 1
# points = []
# while (point_num > 0):
#     points.append((float(data[line_index]), float(data[line_index + 1])))
#     line_index += 2
#     point_num -= 1
# plt.plot([p[0] for p in points], [p[1] for p in points], color='red')


# plt.show()

data = "(43.4132, 25.9563)->(47.25, 29.25)"

data = data.split("->")


data = [eval(data[i]) for i in range(0, len(data))]

# # plot data
plt.plot([p[0] for p in data], [p[1] for p in data], color='green')
# plt.scatter([p[0] for p in data], [p[1] for p in data], color='red')

# data = "(20.5829, 19.5643)->(20.25, 19.25)->(20.75, 18.75)->(21.25, 18.25)->(21.75, 17.75)->(22.25, 17.25)->(22.75, 16.75)->(23.25, 17.25)->(23.75, 17.75)->(24.25, 18.25)->(24.75, 18.75)->(25.25, 19.25)->(25.25, 19.75)->(25.25, 20.25)->(25.25, 20.75)->(25.25, 21.25)->(25.25, 21.75)->(25.25, 22.25)->(25.25, 22.75)->(25.25, 23.25)->(25.25, 23.75)->(25.25, 24.25)->(25.25, 24.75)->(25.25, 25.25)->(25.25, 25.75)->(25.25, 26.25)->(25.25, 26.75)->(25.25, 27.25)->(25.25, 27.75)->(25.25, 28.25)->(24.75, 28.75)->(24.25, 29.25)->(23.75, 29.75)->(23.25, 30.25)"

# data = data.split("->")

# # 22 30.5 21 28.5 plot the line
# plt.plot([22, 22.5], [30.5, 30.5], color='red')

# plt.scatter(26.25, 8.75, marker='o', color='red')
# plt.scatter(26.75, 8.25, marker='o', color='green')
# plt.scatter(26, 9.25, marker='o', color='blue')


# data = [eval(data[i]) for i in range(0, len(data))]

# # plot data
# plt.plot([p[0] for p in data], [p[1] for p in data], color='green')
# plt.scatter([p[0] for p in data], [p[1] for p in data], color='red')
plt.show()
