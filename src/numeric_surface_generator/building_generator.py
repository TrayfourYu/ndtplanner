#!/usr/bin/python3
import numpy as np
import pptk
import pandas as pd
import json
import os
import sys

def addSingleCylinderObs(r0, theta0, h0, radius, height):
    obs_x = []
    obs_y = []
    obs_z = []
    x0 = r0 * np.cos(theta0)
    y0 = r0 * np.sin(theta0)
    z0 = h0
    resolution = 0.2
    size_angle = int(radius*2*np.pi / resolution)+1
    size_z = int(height / resolution)+1
    size_radius = int(radius / resolution)
    for i in range(size_z):
        new_z = z0 + i*resolution
        for j in range(size_angle):
            theta = (resolution*j) / radius
            new_x = x0 + radius*np.cos(theta)
            new_y = y0 + radius*np.sin(theta)
            obs_x.append(new_x)
            obs_y.append(new_y)
            obs_z.append(new_z)
            if i == 0 or i == size_z-1:
                for k in range(1, size_radius):
                    new_x = x0 + resolution*k*np.cos(theta)
                    new_y = y0 + resolution*k*np.sin(theta)
                    obs_x.append(new_x)
                    obs_y.append(new_y)
                    obs_z.append(new_z)
    return obs_x, obs_y, obs_z

def getstairs(x0, y0, z0):
    x = []
    y = [y0]
    z = [z0]
    length = (h+b)*n
    yz_num = int(length/res) + 1

    for i in range(1, yz_num):
        s = i*res
        count = s % (h+b)
        if count <= h:
            z.append(z[-1] + res)
            y.append(y[-1])
        else:
            z.append(z[-1])
            y.append(y[-1] + res)

    x_num = int(l/res) + 1
    y = y * x_num
    z = z * x_num
    for i in range(0, x_num):
        xi = res*i + x0
        tmp = [xi]
        x = x + tmp * yz_num
    return x, y, z

def getRect(length, width):
    x = []
    y = []
    z = []
    x_num = int(width/res) + 1
    y_num = int(length/res) + 1

    for i in range(0, x_num):
        xi = res*i
        tmp = [xi]
        x = x + tmp * y_num

    for i in range(0, y_num):
        yi = res*i
        y.append(yi)
    y = y * x_num

    z = [0] * (x_num * y_num)

    return x, y, z

# 读取参数
basic_dir = os.path.split(os.path.realpath(__file__))[0]
# # setting params:
with open(basic_dir+'/params.json') as f:
    params = json.load(f)

h = 0.1
b = 0.2
l = 1.5
res = 0.02
n = 10

x1, y1, z1 = getstairs(0.0,0.0,0.0)
x2, y2, z2 = getRect(l, 2*l+0.2)
x1 = x1 + x2
y1 = y1 + [i+n*b for i in y2]
z1 = z1 + [i+n*h for i in z2]

x = x1 + [-i+2*l+0.2 for i in x1]
y = y1 + [-i+10*b for i in y1]
z = z1 + [i+n*h for i in z1]

x3 = x
y3 = y
z3 = [i+2*n*h for i in z]

x4 = x
y4 = y
z4 = [i+4*n*h for i in z]

x5, y5, z5 = getRect(20.0, 2.0)
x5 = [i-2.0 for i in x5]
y5 = [i+l-20.0 for i in y5]
z5 = [i+6*n*h for i in z5]

x = x + x3 + x4 + x5
y = y + y3 + y4 + y5
z = z + z3 + z4 + z5

x3, y3, z3 = getRect(35, 2*l+3)
x = x + [i-2.4 for i in x3]
y = y + [i-21 for i in y3]
z = z + z3

x3, y3, z3 = getRect(0.2,20)
n_x = len(x3)
n_y = len(y3)

wall_h = 1
n_h = int(wall_h/res)
z4 = []
for i in range(n_h):
    tmp = [i*res]
    z4 = z4 + tmp*n_x

x4 = [-i for i in y3]*n_h
y4 = [i for i in x3]*n_h

x = x + x4
y = y + y4
z = z + z4

x3, y3, z3 = getRect(0.1,1.2)
n_x = len(x3)
n_y = len(y3)

door_h = 1
n_h = int(door_h/res)
z4 = []
for i in range(n_h):
    tmp = [i*res + 6*n*h]
    z4 = z4 + tmp*n_x

x4 = [i-1.2 for i in x3]*n_h
y4 = [i-5 for i in y3]*n_h

x = x + x4
y = y + y4
z = z + z4

z = [i+1 for i in z]


# if params['with_obs']:
#     obs_x, obs_y, obs_z = addSingleCylinderObs(4, 2*np.pi/3, 1, 1, 3)
#     x = x + obs_x
#     y = y + obs_y
#     z = z + obs_z
#     obs_x, obs_y, obs_z = addSingleCylinderObs(4, 3*np.pi/2, 3*np.pi/2, 0.9, 3)
#     x = x + obs_x
#     y = y + obs_y
#     z = z + obs_z
#     obs_x, obs_y, obs_z = addSingleCylinderObs(6, 5*np.pi/2, 5*np.pi/2, 0.9, 3)
#     x = x + obs_x
#     y = y + obs_y
#     z = z + obs_z
#     obs_x, obs_y, obs_z = addSingleCylinderObs(3, 7*np.pi/2, 7*np.pi/2, 0.9, 3)
#     x = x + obs_x
#     y = y + obs_y
#     z = z + obs_z
#     obs_x, obs_y, obs_z = addSingleCylinderObs(4, 8*np.pi/2, 8*np.pi/2, 2, 3)
#     x = x + obs_x
#     y = y + obs_y
#     z = z + obs_z
# pass

df = pd.DataFrame({'x': x, 'y': y, 'z': z})
file_name = ''
if params['with_obs']:
    file_name = params['file_prefix']+'1/building.csv'
else:
    file_name = params['file_prefix']+'1/building_with_obs.csv'
pass
df.to_csv(file_name, index=False, sep=',')
v = pptk.viewer(df)
