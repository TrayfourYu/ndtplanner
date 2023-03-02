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

def getTrape(h1, h2, width, l):
    x = []
    y = []
    z = []
    x_num = int(width/res) + 1

    for i in range(0, x_num):
        xi = res*i
        zi = -(h2 - h1)/width*xi + h2
        z_num = int(zi/res) + 1
        tmp = [xi]
        x = x + tmp * z_num
        z = z + [i*res for i in range(z_num)]

    y_num = int(l/res) + 1
    for i in range(0, y_num):
        yi = res*i
        tmp = [yi]
        y = y + tmp * len(x)
    
    x = x * y_num
    z = z * y_num

    return x, y, z

# 读取参数
basic_dir = os.path.split(os.path.realpath(__file__))[0]
# # setting params:
with open(basic_dir+'/params.json') as f:
    params = json.load(f)

b = 1.5
l = 10.0
res = 0.02

x, y, z = getRect(l, b)

x3, y3, z3 = getTrape(0.4, 0.1, b, 0.3)
x4, y4, z4 = getTrape(0.1, 0.4, b, 0.3)
x = x + x3
y = y + [i+l/4 for i in y3]
z = z + z3
x = x + x4
y = y + [i+l/4*2 for i in y4]
z = z + z4
x = x + x3
y = y + [i+l/4*3 for i in y3]
z = z + z3

x2 = [i+b for i in y]
y2 = [-i+l for i in x]
x = x + x2
y = y + y2
z = z + z

x2 = [i+b for i in y4]
y2 = [-i+l for i in x4]
x = x + x2
y = y + y2
z = z + z4

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
    file_name = params['file_prefix']+'travel.csv'
else:
    file_name = params['file_prefix']+'travel_with_obs.csv'
pass
df.to_csv(file_name, index=False, sep=',')
v = pptk.viewer(df)
