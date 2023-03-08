#!/usr/bin/python3
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from scipy.signal import savgol_filter
import pandas as pd
import csv
import json
import os
import sys
from geomdl import BSpline
from geomdl import fitting
from geomdl import utilities
from geomdl import exchange
# from geomdl.visualization import VisPlotly
from geomdl.visualization import VisMPL as vis
import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from typing import List
from itertools import chain
import math
font_name = "Source Han Sans CN"
mpl.rcParams['font.family'] = font_name
mpl.rcParams['axes.unicode_minus'] = False

basic_dir = os.path.split(os.path.realpath(__file__))[0]
# # setting params:
with open(basic_dir+'/params.json') as f:
    params = json.load(f)
g = np.array([0, 0, -params['g']])


class State:
    def __init__(self, point, n, alpha, beta, kappa, s, psi, pitch):
        # if abs(np.dot(alpha, n)) > 1e-6:
        # print('tangent dot normal is ', np.dot(alpha, n))
        # assert(abs(np.dot(alpha, n)) < 1e-6)
        # assert(abs(np.dot(alpha, beta)) < 1e-1)
        self.point = point
        self.n = n
        self.alpha = alpha
        self.beta = beta
        self.kappa = kappa
        self.s = s
        self.gamma = np.cross(n, alpha)
        self.psi = psi
        self.pitch = pitch
        self.k_n = np.dot(beta, n)*kappa
        self.k_y = np.dot(beta, self.gamma)*kappa
        self.g_t = np.dot(g, alpha)
        self.g_n = np.dot(g, n)
        self.g_y = np.dot(g, self.gamma)


Path = List[State]

def infimum(lists):
    num = len(lists)
    if num == 1:
        return lists[0]
    elif num == 0:
        return []
    size = len(lists[0])
    result = []
    for i in range(size):
        inf = lists[0][i]
        for j in range(1, num):
            inf = min(inf, lists[j][i])
        result.append(inf)
    return result


def toList(path: Path, type='s'):
    l = []
    if type == 's':
        for state in path:
            l.append(state.s)
    elif type == 'alpha':
        for state in path:
            l.append(state.alpha)
    elif type == 'beta':
        for state in path:
            l.append(state.beta)
    elif type == 'n':
        for state in path:
            l.append(state.n)
    elif type == 'kappa':
        for state in path:
            l.append(state.kappa)
    elif type == 'k_n':
        for state in path:
            l.append(state.k_n)
    elif type == 'k_y':
        for state in path:
            l.append(state.k_y)
    elif type == 'g_n':
        for state in path:
            l.append(state.g_n)
    elif type == 'g_y':
        for state in path:
            l.append(state.g_y)
    elif type == 'g_t':
        for state in path:
            l.append(state.g_t)
    elif type == 'psi':
        for state in path:
            l.append(abs(180.0 / 3.14 *state.psi))
    elif type == 'pitch':
        for state in path:
            l.append(abs(180.0 / 3.14 *state.pitch))
    return l


def preproccess(file):
    datas = pd.read_csv(file)
    points = []
    normals = []
    arcs = []
    num = len(datas['x'])
    s = 0.
    for i in range(num):
        pt = [datas['x'][i], datas['y'][i], datas['z'][i]]
        points.append(pt)
        quat = Rot.from_quat([datas['qx'][i], datas['qy'][i],
                              datas['qz'][i], datas['qw'][i]])
        rot_matrix = quat.as_matrix()
        surface_normal = np.array([rot_matrix[:, 2]]).transpose()
        if i > 0:
            s = s + np.linalg.norm(np.array(pt)-np.array(points[i-1]))
        arcs.append(s)
        normals.append(surface_normal)

    cur = fitting.interpolate_curve(points, degree=6)
    u_array = fitting.compute_params_curve(points)
    file = open('nodes_without_vel.csv', 'w')
    csv_writer = csv.writer(file)
    csv_writer.writerow(['x', 'y', 'z', 'nx', 'ny', 'nz',
                         'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'kappa', 's'])
    for i in range(num):
        actual_pt = np.array([points[i]]).transpose()
        eval = cur.derivatives(u_array[i], 2)
        r = np.array([eval[0]]).transpose()
        dr = np.array([eval[1]]).transpose()
        ddr = np.array([eval[2]]).transpose()
        if np.linalg.norm(actual_pt - r) < 1e-6:
            # print('dr is \n', dr)
            vel = np.linalg.norm(dr)
            alpha = dr / vel
            projection_matrix = np.eye(3) - np.matmul(alpha, alpha.transpose())
            curvature_vec = np.matmul(projection_matrix, ddr) / vel**2
            curvature = np.linalg.norm(curvature_vec)
            kappa2 = abs(np.dot(np.array([0, 0, 1]), np.cross(
                dr[:, 0], ddr[:, 0])) / vel**3)
            if abs(kappa2 - curvature) > 1e-3:
                print('two method has different result: ',
                      curvature, ' and', kappa2)
                assert(abs(kappa2 - curvature) < 1e-6)
            beta = curvature_vec / curvature
            csv_writer.writerow([r[0][0], r[1][0], r[2][0],
                                 normals[0][0], normals[1][0], normals[2][0],
                                 alpha[0][0], alpha[1][0], alpha[2][0],
                                 beta[0][0], beta[1][0], beta[2][0],
                                 curvature, arcs[i]])
        else:
            print(' interpolation is not exact!')
            break
    file.close()


def getAcceleration(v, s):
    size = len(s)
    a = []
    for i in range(size-2):
        a.append((v[i+1]**2 - v[i]**2) / 2 / (s[i+1] - s[i]))
    a.append(a[-1])
    a.append(a[-1])
    assert(len(a) == size)
    return a

def IsSlope(pitch):
    slope = 6.0 / 180.0 * math.pi
    if pitch < slope and pitch > -slope:
        return False
    else:
        return True

StairsEnd = []
def initialize(file):
    datas = pd.read_csv(file)
    points = np.array([datas['x'], datas['y'], datas['z']])
    normals = np.array([datas['nx'], datas['ny'], datas['nz']])
    #sm_normals = savgol_filter(normals, 15, 1)
    alphas = np.array([datas['tx'], datas['ty'], datas['tz']])
    sm_alphas = savgol_filter(alphas, 15, 1)
    betas = np.array([datas['qx'], datas['qy'], datas['qz']])
    #sm_betas = savgol_filter(betas, 15, 1)
    arcs = datas['s']
    kappas = datas['kappa']
    psi = datas['psi']
    sm_psi = savgol_filter(psi, 15, 1)
    pitch = datas['theta']
    #smooth pitch
    sm_pitch = savgol_filter(pitch, 15, 1)
    path = []
    size = len(arcs)
    for i in range(size):
        path.append(State(points[:, i], normals[:, i],
                          sm_alphas[:, i], betas[:, i], kappas[i], arcs[i], sm_psi[i], sm_pitch[i]))
        isend = False
        if not IsSlope(pitch[i]):
            isend = True
            for j in range(1,4):
                if not IsSlope(pitch[max(0,i - j)]):
                    isend = False
                    break
            for j in range(1,3):
                if IsSlope(pitch[min(size-1, i + j)]):
                    isend = False
                    break   
        if isend:
            StairsEnd.append(i)               
    return path


def getJerk(s1, s2, s3, v1, v2, v3):
    # v = a*s^3 + b*s + c
    # j = 2a*v + (2as+b)^*v
    # a1 = (v2**2 - v1**2) / 2 / (s2 - s1)
    # a2 = (v3**2 - v2**2) / 2 / (s3 - s2)
    # return (a2 - a1) / (s3 - s2)
    coe1 = s1**2 - s1*s2 - s1*s3 + s2*s3
    coe2 = s1*s2 - s1*s3 - s2**2 + s2*s3
    coe3 = s1*s2 - s1*s3 - s2*s3 + s3**2
    # a = v1 / coe1 - v2 / coe2 + v3 / coe3
    # b = -v1*(s2+s3) / coe1 + v2*(s1+s3)/coe2 - v3*(s1+s2)/coe3
    # # c = v1*s2*s3 / coe1 - v2*s1*s3 / coe2 + v3*s1*s2 / coe3
    # return 2*a*v2**2 + (2*a*s2+b)**2 * v2
    return 2*v3 / coe3 + 2*v1 / coe1 - 2*v2 / coe2


def accRangeFromSlidingLimit(state: State, v):
    miu = params['miu']
    # c1 = (state.k_n**2 * miu**2 - state.k_y**2)
    # c2 = 2*(-state.k_n*state.g_n*miu**2 + state.k_y * state.g_y)
    c3 = (state.g_n**2 * miu**2 - state.g_y**2)
    #Delta = c1*v**4 + c2*v**2 + c3
    Delta = c3
    if Delta < 1e-1 and Delta > -1e-1:
        Delta = np.abs(Delta)
    elif Delta < 0:
        raise Exception(
            'unstable surface---- no aviliable acceleration under given vel profile!')
    acc = min(params['acc_comfort_max'], state.g_t -
              params['Cw']*v**2 + np.sqrt(Delta))
    dcc = max(params['acc_comfort_min'], state.g_t -
              params['Cw']*v**2 - np.sqrt(Delta))
    return {'max': acc, 'min': dcc}


def accRangeFromEngineLimit(state: State, v):
    acc = params['acc_engine_max'] + state.g_t - params['Cw']*v**2
    dcc = params['acc_engine_min'] + state.g_t - params['Cw']*v**2
    return {'max': acc, 'min': dcc}

def accRangeFromRolloverLimit(state: State, params):
    h = params['h']
    rear = params['rear']
    p = abs(state.pitch)
    acc = params['g'] * (math.cos(p) * rear / h - math.sin(p))
    dcc = params['g'] * (-math.cos(p) * rear / h + math.sin(p))
    return {'max': acc, 'min': dcc}

def smoothnessLimit(path: Path, v, j_max):
    size = len(path)
    for i in range(1, size-1):
        coe1 = path[i-1].s*path[i].s + path[i-1].s * \
            path[i+1].s - path[i].s*path[i+1].s - path[i-1].s**2
        coe2 = path[i-1].s*path[i].s - path[i-1].s * \
            path[i+1].s + path[i].s*path[i+1].s - path[i].s**2
        coe3 = path[i-1].s*path[i].s - path[i-1].s * \
            path[i+1].s - path[i].s*path[i+1].s + path[i+1].s**2
        j = 2*v[i+1] / coe3 - 2*v[i-1] / coe1 - 2*v[i] / coe2
        if j > j_max:
            v[i] = (-j_max + 2*v[i+1] / coe3 - 2*v[i-1] / coe1) * coe2 / 2
        elif j < -j_max:
            v[i] = (j_max + 2*v[i+1] / coe3 - 2*v[i-1] / coe1) * coe2 / 2
    return v


def velProfileFromcontactLimit(path: Path, params):
    size = len(path)
    v_upper = params['v_max']*np.ones(size)
    for i in range(size):
        state = path[i]
        if state.k_n * state.g_n > 0:
            v_upper[i] = min(v_upper[i], np.sqrt(state.g_n / state.k_n))
        if i > 0 and v_upper[i] - v_upper[i-1] > 2:
            v_upper[i] = v_upper[i-1] + 2
    return {'upper': v_upper}

def velProfileFromStairsEndLimit(path: Path, params):
    size = len(path)
    v_upper = params['v_max']*np.ones(size)
    for i in range(size):
        if StairsEnd.count(i):
            v_upper[i] = 0.1
    return {'upper': v_upper}

def velProfileFromSlidingLimit(path: Path, params):
    size = len(path)
    V_upper = params['v_max']*np.ones(size)
    V_lower = np.zeros(size)
    miu = params['miu']
    for i in range(size):
        state = path[i]
        c1 = (state.k_n**2 * miu**2 - state.k_y**2)
        c2 = 2*(-state.k_n*state.g_n*miu**2 + state.k_y * state.g_y)
        c3 = (state.g_n**2 * miu**2 - state.g_y**2)
        Delta = c2**2 - 4*c1*c3
        if Delta >= 0:
            if c1 < 0:
                c3 = c3 - state.g_t**2
                # Delta = c2**2 - 4*c1*c3
                # if Delta < 0:
                #     raise Exception(
                #         'unstable surface----- no aviliable speed section get possitive acceleration!')
                v_lower = (-c2 + np.sqrt(Delta)) / 2 / c1
                v_upper = (-c2 - np.sqrt(Delta)) / 2 / c1
                if v_upper < 0:
                    print('[', i, ']: coef(', c1, ', ', c2, ', ', c3,
                          '), vel section:[', v_lower, ', ', v_upper, ']\n')
                    raise Exception(
                        'unstable surface----- max speed is negative!')
                V_upper[i] = min(V_upper[i], np.sqrt(v_upper))
                if v_lower > 0:
                    V_lower[i] = np.sqrt(v_lower)
            # elif c1 == 0:
            #     continue
            else:
                # c3 = c3 - state.g_t**2
                # Delta = c2**2 - 4*c1*c3
                v_upper = (-c2 + np.sqrt(Delta)) / 2 / c1
                v_lower = (-c2 - np.sqrt(Delta)) / 2 / c1
                if v_lower > 0:
                    # print('>>>>>>>[', i, ']:',
                    #       'max speed limit by v_lower:', np.sqrt(v_lower))
                    V_upper[i] = min(np.sqrt(v_lower), V_upper[i])
                elif v_upper > 0:
                    # print('>>>>>>>[', i, ']:',
                    #       'min speed limit by v_upper:', np.sqrt(v_upper))
                    V_lower[i] = np.sqrt(v_upper)
        # elif c1 > 0:
        #     c3 = c3 - state.g_t**2
        #     Delta = c2**2 - 4*c1*c3
        #     if Delta > 0:
        #         v_upper = (-c2 + np.sqrt(Delta)) / 2 / c1
        #         v_lower = (-c2 - np.sqrt(Delta)) / 2 / c1
        #         if v_lower > 0:
        #             # print('>>>>>>>2[', i, ']:',
        #             #       'max speed limit by v_lower:', np.sqrt(v_lower))
        #             V_upper[i] = min(np.sqrt(v_lower), V_upper[i])
        #         elif v_upper > 0:
        #             # print('>>>>>>>2[', i, ']:',
        #             #       'min speed limit by v_upper:', np.sqrt(v_upper))
        #             V_lower[i] = np.sqrt(v_upper)
        elif c1 < 0:
            raise Exception(
                'unstable surface----- no aviliable speed section!')
        if i > 0 and V_upper[i] - V_upper[i-1] > 1:
            V_upper[i] = V_upper[i-1] + 2
        V_upper[i] = min(V_upper[i], params['v_max'])

    return {'upper': V_upper, 'lower': V_lower}

# def velProfileFromRolloverLimit(path: Path, params):
#     size = len(path)
#     v_upper = params['v_max']*np.ones(size)
#     v_lower = np.zeros(size)
#     h = params['h']
#     b = params['b']
#     psi_max = params['psi_max']
#     for i in range(size):
#         state = path[i]
#         if abs(state.psi) >= psi_max:
#             c1 = state.k_n*b - state.k_y*h
#             c2 = state.g_y*h - state.g_n*b
#             if c1 < 0 and c2 >= 0:
#                 v_upper[i] = min(v_upper[i], np.sqrt(c2 / (-c1)))
#             elif c1 > 0 and c2 < 0:
#                 v_lower[i] = np.sqrt(-c2 / c1)
#                 print('[%d]:min speed limit %f' % (i, v_lower[i]))
#             # elif abs(c1)==0:
#             #     continue
#             elif c1 <= 0 and c2 <= 0:
#                 print('[%d]:c1 and c2 is (%f, %f), kappa, kn, ky is(%f,%f,%f)' %
#                       (i, c1, c2, state.kappa, state.k_n, state.k_y))
#                 raise Exception(
#                     'unstable surface----- no aviliable speed section for tip-over constraint!')
#     return {'upper': v_upper, 'lower': v_lower}


def accProfileFromSlidingLimit(path: Path, params, v):
    acc = []
    dcc = []
    size = len(path)
    for i in range(size):
        limit = accRangeFromSlidingLimit(path[i], v[i])
        acc.append(limit['max'])
        dcc.append(limit['min'])
    return dcc, acc


def accProfileFromEngineLimit(path: Path, params, v):
    acc = []
    dcc = []
    size = len(path)
    for i in range(size):
        limit = accRangeFromEngineLimit(path[i], v[i])
        acc.append(limit['max'])
        dcc.append(limit['min'])
    return dcc, acc

def accProfileFromRollOverLimit(path: Path, params):
    acc = []
    size = len(path)
    for i in range(size):
        limit = accRangeFromRolloverLimit(path[i], params)
        acc.append(limit['max'])
    return acc

def speedPlanning(path: Path, params):
    size = len(path)
    miu = params['miu']
    V0 = np.ones(size) * params['v_max']
    Vf = np.zeros(size)
    # endpoints constraints:
    V0[0] = params['v0']
    V0[-1] = params['ve']

    #contact_limit = velProfileFromcontactLimit(path, params)
    sliding_limit = velProfileFromSlidingLimit(path, params)
    #rollover_limit = velProfileFromRolloverLimit(path, params)
    stairsend_limit = velProfileFromStairsEndLimit(path, params)

    #V_contact_upper = contact_limit['upper']
    V_sliding_upper = sliding_limit['upper']
    #V_rollover_upper = rollover_limit['upper']
    V_sliding_lower = sliding_limit['lower']
    #V_rollover_lower = rollover_limit['lower']
    V_stairsend_upper = stairsend_limit['upper']

    count = 0
    while np.linalg.norm(V0 - Vf) > 1e-6 and count < size:
        Vf = V0.copy()
        # V0 = smoothnessLimit(path, V0, params['j_max'])
        for i in range(1, size-1):
            #V0[i] = min(V0[i], V_contact_upper[i])
            V0[i] = min(V0[i], V_sliding_upper[i])
            #V0[i] = min(V0[i], V_rollover_upper[i])
            V0[i] = min(V0[i], V_stairsend_upper[i])
        for i in range(1, size-1):
            # acceleration imit:
            slide_limit = accRangeFromSlidingLimit(path[i-1], V0[i-1])
            # engine_limit = accRangeFromEngineLimit(path[i-1], V0[i-1])
            rollover_limit = accRangeFromRolloverLimit(path[i-1], params)
            a_max = max(0, min(slide_limit['max'], 
                                rollover_limit['max'], params['acc_comfort_max']))
            V0[i] = min(V0[i], np.sqrt(V0[i-1]**2 + 2 *
                                       a_max * (path[i].s - path[i-1].s)))

            # deceleration limit:
            slide_limit = accRangeFromSlidingLimit(path[i], V0[i])
            # engine_limit = accRangeFromEngineLimit(path[i], V0[i])
            d_max = - \
                max(slide_limit['min'],
                    params['acc_comfort_min'])
            V0[i] = min(V0[i], np.sqrt(V0[i+1]**2 + 2 *
                                       d_max * (path[i+1].s - path[i].s)))
        count = count + 1

    print('speed planner solver over ,iteration count is ', count, '/', 2*size)
    return V0.tolist()


def fittingVel(s, v):
    size = len(s)
    ctrlpts = []
    for i in range(size):
        if i == 0:
            ctrlpts.append([s[i], v[i]])
        elif abs(ctrlpts[-1][0] - s[i]) >= 0.5:
            ctrlpts.append([s[i], v[i]])
    # fitting:
    curve = BSpline.Curve()
    curve.degree = 10
    curve.ctrlpts = ctrlpts
    curve.knotvector = utilities.generate_knot_vector(
        curve.degree, len(curve.ctrlpts))
    new_s = []
    new_v = []
    new_a = []
    new_j = []
    for u in np.linspace(0, 1, 5e3):
        eval = curve.derivatives(u, 2)
        si = eval[0][0]
        vi = eval[0][1]
        dsi = eval[1][0]
        dvi = eval[1][1]
        ddsi = eval[2][0]
        ddvi = eval[2][1]
        ai = dvi / dsi * vi
        ji = (ddvi / dsi**2 - dvi / ddsi**3) * vi**2 + ai * dvi / dsi
        new_v.append(vi)
        new_s.append(si)
        new_a.append(ai)
        new_j.append(ji)
    new_a[0] = new_a[1]
    return new_s, new_v, new_a, new_j


# preproccess('terrain_demo_result.csv')
data_path = basic_dir + '/../../../cache/'
path_file = params['file_prefix']+'.csv'
path = initialize(data_path+path_file)

plt.figure()
roll, pitch = toList(path, 'psi'), toList(path, 'pitch')
colors = [math.sqrt(roll[i]**2 + pitch[i]**2) for i in range(len(roll))]
plt.scatter(pitch,roll, s=10, c=colors, cmap="summer")
plt.xlim(0,10)
plt.ylim(0,10)
plt.xlabel("俯仰角 (度)")
plt.ylabel("侧倾角 (度)")
plt.show()

#vel_contact = velProfileFromcontactLimit(path, params)
vel_slide = velProfileFromSlidingLimit(path, params)
vel_stairsend = velProfileFromStairsEndLimit(path, params)
#vel_rollover = velProfileFromRolloverLimit(path, params)

vel = speedPlanning(path, params)
s = toList(path, 's')
# new_s, new_v, new_a, new_j = fittingVel(s, vel)
acc_slide_min, acc_slide_max = accProfileFromSlidingLimit(path, params, vel)
# acc_engine_min, acc_engine_max = accProfileFromEngineLimit(path, params, vel)
acc_rollover_max = accProfileFromRollOverLimit(path, params)
acc_comfort_min = (params['acc_comfort_min']*np.ones(len(path))).tolist()
acc_comfort_max = (params['acc_comfort_max']*np.ones(len(path))).tolist()
acc_real = getAcceleration(vel, s)

##################plot#########################
if(params['is_plot']):
    fig1, ax = plt.subplots(2, 1)
    # ax1 = fig1.gca()
    # ax[0].plot(s, vel_contact['upper'], 'r--', s,
    #            vel_slide['upper'], 'b--', s, vel, 'b')
    ax[0].plot( s, vel_slide['upper'], 'r--', 
                s, vel_stairsend['upper'], 'y--',
                s, vel, 'b')
    #ax[0].plot(s, vel_rollover['upper'], 'y-.', s, vel_rollover['lower'], 'm-.')
    ax[0].plot()
    ax[0].set_title('速度曲线生成结果')
    ax[0].set_xlabel('弧长 (m)')
    ax[0].set_ylabel('速度 (m/s)')
    ax[0].legend(['曲率约束', '冲击约束','速度曲线'], loc='best')
    #ax[0].legend(['contact', 'sliding', 'actual', 'rollover upper', 'rollover lower'], loc='best')
    ax[0].grid(alpha=0.6, linestyle='--', color='gray')

    #ax[1].plot(s, acc_slide_min, color='lightcoral', linestyle='--')
    ax[1].plot(s, acc_slide_max, color='darkred', linestyle='--')
    ax[1].plot(s, acc_rollover_max, color='orangered', linestyle='--')
    # ax[1].plot(s, acc_engine_min, color='tan', linestyle='--')
    # ax[1].plot(s, acc_engine_max, color='orangered', linestyle='--')
    # ax[1].plot(s, acc_comfort_min, color='bisque', linestyle='--')
    # ax[1].plot(s, acc_comfort_max, color='burlywood', linestyle='--')
    ax[1].plot(s, acc_real, color='deepskyblue')
    # ax[1].set_title('curvature profile')
    # ax[1].legend(['sliding_lower', 'sliding_upper', 'engine_lower',
    #               'engine_upper', 'comfort_lower', 'comfort_upper', 'actual', ], loc='best')
    ax[1].legend(['驱动约束', '倾翻约束',  '加速度曲线' ], loc='best')
    ax[1].set_xlabel('弧长 (m)')
    ax[1].set_ylabel('加速度 (m/s$^2$)')
    ax[1].grid(alpha=0.6, linestyle='--', color='gray')

    # plt.figure()
    # plt.plot(s, toList(path, 'kappa'))
    # # ax[1].set_title('curvature profile')
    # plt.xlabel('s (m)')
    # plt.ylabel('curvature (m$^{-1}$)')
    # plt.grid(alpha=0.6, linestyle='--', color='gray')

    # plt.figure()
    # plt.plot(s,  toList(path, 'alpha'))
    # plt.title('alpha')

    # plt.figure()
    # plt.plot(s,  toList(path, 'beta'))
    # plt.title('beta')

    plt.figure()
    plt.plot(s,  toList(path, 'pitch'))
    plt.title('pitch')

    # 3d
    # fig2 = plt.figure()
    # ax1 = fig2.add_subplot(111, projection="3d")
    # x = np.arange(10, 50, 10)
    # y = np.arange(10, 50, 10)
    # xx, yy = np.meshgrid(x, y)
    # plot_x, plot_y = xx.ravel(), yy.ravel()

    # count = [[0]*4]*4
    # for state in path:
    #     roll = abs(state.psi * 180.0 / math.pi)
    #     pitch = abs(state.pitch * 180.0 / math.pi)
    #     index1 = math.floor(roll/10)
    #     index2 = math.floor(pitch/10)
    #     if index1<4 and index2<4:
    #         count[index1][index2] = count[index1][index2] + 1
    # rate = list(chain(*count))
    # summ = sum(rate)
    # rate = [100*x/summ for x in rate]

    # bottom = np.zeros_like(rate)
    # ax1.bar3d(plot_x, plot_y, bottom, 10, 10, rate, shade = True)



# save data in file:
vel_upper = infimum([vel_slide['upper'], vel_stairsend['upper']])
acc_upper = infimum([acc_slide_max, acc_rollover_max])
file = pd.DataFrame({'s': s, 'v0': vel, 'v_upper':vel_upper, 'v_slide': vel_slide['upper'],
                     'vel_stairsend': vel_stairsend['upper'],  
                     'a0': acc_real,'acc_upper':acc_upper, 
                     'acc_slide': acc_slide_max,'acc_rollover': acc_rollover_max})
                    #  'k_n': toList(path, 'k_n'),
                    #  'k_y': toList(path, 'k_y'), 'g_n': toList(path, 'g_n'),
                    #  'g_y': toList(path, 'g_y'), 'g_t': toList(path, 'g_t')})
output_file = params['file_prefix']+'_initial_speed.csv'
file.to_csv(data_path+output_file, index=False, sep=',')
