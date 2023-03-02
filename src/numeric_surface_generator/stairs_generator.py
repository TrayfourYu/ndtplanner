#!/usr/bin/python3
import numpy as np
import pptk
import pandas as pd
import json
import os
import sys

def addSingleCylinderObs(x0, y0, z0, radius, height):
    obs_x = []
    obs_y = []
    obs_z = []
    
    resolution = 0.02
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

def addSingleCylinderObsWithoutPlane(x0, y0, z0, radius, height):
    obs_x = []
    obs_y = []
    obs_z = []
    
    resolution = 0.02
    size_angle = int(radius*2*np.pi / resolution)+1
    size_z = int(height / resolution)+1
    for i in range(size_z):
        new_z = z0 + i*resolution
        for j in range(size_angle):
            theta = (resolution*j) / radius
            new_x = x0 + radius*np.cos(theta)
            new_y = y0 + radius*np.sin(theta)
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
l = 1.8
res = 0.02
n = 10

x1, y1, z1 = getstairs(0.0,0.0,0.0)
x2, y2, z2 = getstairs(0.0,0.0,0.0)
x = x1 + [-i for i in y2]
y = y1 + [i+y1[-1] for i in x2]
z = z1 + [i+z1[-1] for i in z2]

x3, y3, z3 = getRect(7.0, 7.0)
x = x + [i-5 for i in x3]
y = y + [i-2 for i in y3]
z = z + z3

x4, y4, z4 = getRect(l, l)
x = x + x4
y = y + [i+n*b for i in y4]
z = z + [i+n*h for i in z4]

x5, y5, z5 = getRect(10*b+l+1, l)
x = x + [i-n*b-l for i in x5]
y = y + [i-1 for i in y5]
z = z + [i+2*n*h for i in z5]
if params['with_obs']:
    obs_x, obs_y, obs_z = addSingleCylinderObsWithoutPlane(l/2, 10*b+l/2, 0, 0.5, 10*h)
    x = x + obs_x
    y = y + obs_y
    z = z + obs_z
    obs_x, obs_y, obs_z = addSingleCylinderObsWithoutPlane(-10*b-l/2, 10*b+l/2, 0, 0.5, 20*h)
    x = x + obs_x
    y = y + obs_y
    z = z + obs_z
    obs_x, obs_y, obs_z = addSingleCylinderObsWithoutPlane(-10*b-l/2, l/2-1, 0, 0.5, 20*h)
    x = x + obs_x
    y = y + obs_y
    z = z + obs_z
    obs_x, obs_y, obs_z = addSingleCylinderObsWithoutPlane(-5*b, 5*b, 0, 0.1, 0.5)
    x = x + obs_x
    y = y + obs_y
    z = z + obs_z
    obs_x, obs_y, obs_z = addSingleCylinderObsWithoutPlane(-5*b, 7.5*b+1, 0, 0.1, 1)
    x = x + obs_x
    y = y + obs_y
    z = z + obs_z
    obs_x, obs_y, obs_z = addSingleCylinderObsWithoutPlane(-7.5*b-1, 5*b, 0, 0.1, 0.5)
    x = x + obs_x
    y = y + obs_y
    z = z + obs_z
    obs_x, obs_y, obs_z = addSingleCylinderObsWithoutPlane(-10*b-l-0.5, -0.5, 0, 0.2, 1)
    x = x + obs_x
    y = y + obs_y
    z = z + obs_z
    obs_x, obs_y, obs_z = addSingleCylinderObsWithoutPlane(-5*b+0.6, 0, 0, 0.35, 1)
    x = x + obs_x
    y = y + obs_y
    z = z + obs_z
    obs_x, obs_y, obs_z = addSingleCylinderObsWithoutPlane(0, -0.6, 0, 0.35, 0.5)
    x = x + obs_x
    y = y + obs_y
    z = z + obs_z
    #
    obs_x, obs_y, obs_z = addSingleCylinderObs(-7.5*b-0.8, 5*b, 20*h, 0.3, 0.35)
    x = x + obs_x
    y = y + obs_y
    z = z + obs_z
    obs_x, obs_y, obs_z = addSingleCylinderObs(-10*b-l+0.4, 10*b+l/2, 20*h, 0.35, 0.3)
    x = x + obs_x
    y = y + obs_y
    z = z + obs_z

    obs_x, obs_y, obs_z = addSingleCylinderObs(-5*b, 7.5*b+0.9, 15*h, 0.2, 0.3)
    x = x + obs_x
    y = y + obs_y
    z = z + obs_z
    obs_x, obs_y, obs_z = addSingleCylinderObs(1.0, 10*b+l-0.6, 10*h, 0.3, 0.3)
    x = x + obs_x
    y = y + obs_y
    z = z + obs_z
    obs_x, obs_y, obs_z = addSingleCylinderObs(0.4, 7.5*b, 7.5*h, 0.2, 0.3)
    x = x + obs_x
    y = y + obs_y
    z = z + obs_z
    obs_x, obs_y, obs_z = addSingleCylinderObs(1.4, 2.5*b, 2.5*h, 0.2, 0.3)
    x = x + obs_x
    y = y + obs_y
    z = z + obs_z
pass

z = [i+1 for i in z]

df = pd.DataFrame({'x': x, 'y': y, 'z': z})
file_name = ''
if params['with_obs']:
    file_name = params['file_prefix']+'1/stairs.csv'
else:
    file_name = params['file_prefix']+'stairs_with_obs.csv'
pass
df.to_csv(file_name, index=False, sep=',')
v = pptk.viewer(df)
