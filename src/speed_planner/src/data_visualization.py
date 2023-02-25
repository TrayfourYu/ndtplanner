import pandas as pd
import numpy as np
import os
import sys
import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

basic_dir = os.path.split(os.path.realpath(__file__))[0]
data_folder = basic_dir + '/../data/'


def normalizeAngle(angle, radian_measure = True):
    if radian_measure:
        while angle > np.pi:
            angle = angle - 2*np.pi
        while angle < -np.pi:
            angle = angle + 2*np.pi
    else:
        while angle > 180:
            angle = angle - 360
        while angle < -180:
            angle = angle + 360
    return angle

def getAcc(v, s):
    size = len(s)
    a = []
    for i in range(size-2):
        a.append((v[i+1]**2 - v[i]**2) / 2 / (s[i+1] - s[i]))
    a.append(a[-1])
    a.append(a[-1])
    assert(len(a) == size)
    return a


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


def supremum(lists):
    num = len(lists)
    if num == 1:
        return lists[0]
    elif num == 0:
        return []
    size = len(lists[0])
    result = []
    for i in range(size):
        sup = lists[0][i]
        for j in range(1, num):
            sup = max(sup, lists[j][i])
        result.append(sup)
    return result


def accProfileFromSlidingLimit(params, v, kn, ky, gt, gn, gy):
    acc = []
    dcc = []
    size = len(v)
    miu = params['miu']
    for i in range(size):
        c1 = (kn[i]**2 * miu**2 - ky[i]**2)
        c2 = 2*(-kn[i]*gn[i]*miu**2 + ky[i] * gy[i])
        c3 = (gn[i]**2 * miu**2 - gy[i]**2)
        Delta = c1*v[i]**4 + c2*v[i]**2 + c3
        if Delta < 1e-1 and Delta > -1e-1:
            Delta = np.abs(Delta)
        elif Delta < 0:
            raise Exception(
                'unstable surface---- no aviliable acceleration under given vel profile!')
        acc.append(min(9.8, gt[i] + np.sqrt(Delta) - params['Cw']*v[i]**2))
        dcc.append(max(-9.8, gt[i] - np.sqrt(Delta) - params['Cw']*v[i]**2))
    return dcc, acc


def accProfileFromEngineLimit(params, v, gt):
    acc = []
    dcc = []
    size = len(v)
    for i in range(size):
        acc.append(min(9.8, gt[i] - params['Cw']*v[i]
                       ** 2 + params['acc_engine_max']))
        dcc.append(max(-9.8, gt[i] - params['Cw']*v[i]
                       ** 2 + params['acc_engine_min']))
    return dcc, acc


def plotIterationResult(params, datas):
    kn = datas['k_n']
    ky = datas['k_y']
    gn = datas['g_n']
    gy = datas['g_y']
    gt = datas['g_t']
    v_slide = datas['v_slide']
    v_contact = datas['v_contact']
    v_rollover_upper = datas['v_rollover_upper']
    v_rollover_lower = datas['v_rollover_lower']
    s = datas['s']
    v0 = datas['v0']
    # 速度曲线
    plt.rcParams['font.sans-serif'] = ['simhei']  # 指定默认字体
    plt.rcParams['axes.unicode_minus'] = False  # 用来正常显示负号
    plt.figure(figsize=(8, 5))
    plt.plot(s, v_slide, color='black', linestyle='--')
    plt.plot(s, v_contact, color='red', linestyle='-.')
    plt.plot(s, v_rollover_upper, color='orange', linestyle='--')
    # plt.plot(s, v_rollover_lower, color='darkkhaki', linestyle='--')
    plt.plot(s, v0, color='g')
    plt.grid(alpha=0.4, linestyle='--', color='gray')
    plt.xlabel('弧长$\,$(m)', fontsize=18)
    plt.ylabel('速度$\,$(m/s)', fontsize=18)
    plt.xticks(fontsize=20)
    plt.yticks(fontsize=20)
    plt.legend(['侧滑约束速度上限', '地面接触约束速度上限', '侧翻约束速度上限',
                '迭代算法结果'], loc='best', fontsize=18)
    # 加速度曲线
    plt.figure(figsize=(8, 5))
    a_slide_min, a_slide_max = accProfileFromSlidingLimit(
        params, v0, kn, ky, gt, gn, gy)
    a_engine_min, a_engine_max = accProfileFromEngineLimit(params, v0, gt)
    a_comfort_min = (params['acc_comfort_min']*np.ones(len(s))).tolist()
    a_comfort_max = (params['acc_comfort_max']*np.ones(len(s))).tolist()
    a_real_min = supremum([a_slide_min, a_engine_min, a_comfort_min])
    a_real_max = infimum([a_slide_max, a_engine_max, a_comfort_max])
    acc0 = getAcc(v0, s)
    plt.plot(s, a_slide_min, color='black', alpha=0.98, linestyle='--')
    plt.plot(s, a_slide_max, color='rosybrown', alpha=0.98, linestyle='--')
    plt.plot(s, a_engine_min, color='orangered', alpha=0.98, linestyle='-.')
    plt.plot(s, a_engine_max, color='slategrey', alpha=0.98, linestyle='-.')
    plt.plot(s, a_comfort_min, color='m', alpha=0.98, linestyle='--')
    plt.plot(s, a_comfort_max, color='b', alpha=0.98, linestyle='--')
    plt.plot(s, a_real_min, color='lawngreen', alpha=0.79, linewidth=3)
    plt.plot(s, a_real_max, color='yellow', alpha=0.79, linewidth=3)
    plt.plot(s, acc0, color='g')
    plt.grid(alpha=0.4, linestyle='--', color='gray')
    plt.xlabel('弧长$\,$(m)', fontsize=18)
    plt.ylabel('加速度$\,$(m/s$^2$)', fontsize=18)
    plt.xticks(fontsize=20)
    plt.yticks(fontsize=20)
    legend = plt.legend(['侧滑约束加速度下限', '侧滑约束加速度上限', '制动力约束加速度下限',
                         '驱动力约束加速度上限', '舒适度约束加速度下限', '舒适度约束加速度上限',
                         '实际加速度下限', '实际加速度上限', '实际加速度'], loc='upper right', fontsize=18)
    svg = data_folder + params['file_prefix'] + '_acc0.svg'
    plt.savefig(svg)


def plotOptimizedResult(params, datas):
    kn = datas['k_n']
    ky = datas['k_y']
    gn = datas['g_n']
    gy = datas['g_y']
    gt = datas['g_t']
    v_slide = datas['v_slide']
    v_contact = datas['v_contact']
    v_rollover_upper = datas['v_rollover_upper']
    v_rollover_lower = datas['v_rollover_lower']
    s = datas['s']
    v0 = datas['v0']
    opt_v = datas['vel']
    # 速度曲线
    plt.rcParams['font.sans-serif'] = ['simhei']  # 指定默认字体
    plt.rcParams['axes.unicode_minus'] = False  # 用来正常显示负号
    plt.figure(figsize=(8, 5))
    plt.plot(s, v_slide, color='black', linestyle='-.')
    plt.plot(s, v_contact, color='red', linestyle='--')
    plt.plot(s, v_rollover_upper, color='orange', linestyle='--')
    # plt.plot(s, v_rollover_lower, color='darkkhaki', linestyle='--')
    plt.plot(s, v0, color='purple')
    plt.plot(s, opt_v, color='g')
    plt.grid(alpha=0.4, linestyle='--', color='gray')
    plt.xlabel('弧长$\,$(m)', fontsize=18)
    plt.ylabel('速度$\,$(m/s)', fontsize=18)
    plt.xticks(fontsize=20)
    plt.yticks(fontsize=20)
    plt.legend(['侧滑约束速度上限', '地面接触约束速度上限', '侧翻约束速度上限',
                '迭代算法结果', '优化结果'], loc='best', fontsize=18)
    svg = data_folder + params['file_prefix'] + '_speed.svg'
    plt.savefig(svg)
    # 加速度曲线
    plt.figure(figsize=(8, 5))
    a_slide_min, a_slide_max = accProfileFromSlidingLimit(
        params, opt_v, kn, ky, gt, gn, gy)
    a_engine_min, a_engine_max = accProfileFromEngineLimit(params, opt_v, gt)
    a_comfort_min = (params['acc_comfort_min']*np.ones(len(s))).tolist()
    a_comfort_max = (params['acc_comfort_max']*np.ones(len(s))).tolist()
    a_real_min = supremum([a_slide_min, a_engine_min, a_comfort_min])
    a_real_max = infimum([a_slide_max, a_engine_max, a_comfort_max])
    opt_acc = datas['acc']
    plt.plot(s, a_slide_min, color='black', alpha=0.98, linestyle='--')
    plt.plot(s, a_slide_max, color='rosybrown', alpha=0.98, linestyle='--')
    plt.plot(s, a_engine_min, color='orangered', alpha=0.98, linestyle='-.')
    plt.plot(s, a_engine_max, color='slategrey', alpha=0.98, linestyle='-.')
    plt.plot(s, a_comfort_min, color='m', alpha=0.98, linestyle='--')
    plt.plot(s, a_comfort_max, color='b', alpha=0.98, linestyle='--')
    plt.plot(s, a_real_min, color='lawngreen', alpha=0.79, linewidth=3)
    plt.plot(s, a_real_max, color='yellow', alpha=0.79, linewidth=3)
    plt.plot(s, opt_acc, color='g')
    plt.grid(alpha=0.4, linestyle='--', color='gray')
    plt.xlabel('弧长$\,$(m)', fontsize=18)
    plt.ylabel('加速度$\,$(m/s$^2$)', fontsize=18)
    plt.xticks(fontsize=20)
    plt.yticks(fontsize=20)
    plt.legend(['侧滑约束加速度下限', '侧滑约束加速度上限', '制动力约束加速度下限',
                '驱动力约束加速度上限', '舒适度约束加速度下限', '舒适度约束加速度上限',
                '实际加速度下限', '实际加速度上限', '实际加速度'], loc='upper right', fontsize=18)
    svg = data_folder + params['file_prefix'] + '_acc.svg'
    plt.savefig(svg)


def plotPathGeometry(params, datas):
    # 姿态角
    plt.figure(figsize=(8, 5))
    s = datas['s']
    psi = (np.array(datas['psi']) * 180 / np.pi).tolist()
    theta = (np.array(datas['theta']) * 180 / np.pi).tolist()
    phi = np.array(datas['phi']) - np.ones(len(s))*datas['phi'][0]
    phi = (phi * 180 / np.pi).tolist()
    for i in range(len(phi)):
        phi[i] = normalizeAngle(phi[i], False)
    psi_max = (np.ones(len(s))*params['psi_max'] * 180/np.pi).tolist()
    psi_min = (-np.ones(len(s))*params['psi_max']*180/np.pi).tolist()
    plt.plot(s, psi_max, color='lawngreen', linestyle='--', linewidth=3)
    plt.plot(s, psi_min, color='yellow', linestyle='--', linewidth=3)
    plt.plot(s, psi, color='red')
    plt.plot(s, theta, color='green')
    # plt.plot(s, phi, color='blue')
    plt.grid(alpha=0.4, linestyle='--', color='gray')
    plt.xlabel('弧长$\,$(m)', fontsize=18)
    plt.ylabel('角度$\,$($^\circ$)', fontsize=18)
    # plt.legend(['右倾侧翻约束激活阈值', '左倾侧翻约束激活阈值', '滚转角', '俯仰角', '航向角'], fontsize=18)
    plt.legend(['右倾侧翻约束激活阈值', '左倾侧翻约束激活阈值', '滚转角', '俯仰角'], fontsize=18)
    plt.xticks(fontsize=18)
    plt.yticks(fontsize=18)
    # svg = data_folder + params['file_prefix'] + '_euler_angle.svg'
    svg = data_folder + params['file_prefix'] + '_roll_and_pitch.svg'
    plt.savefig(svg)
    # 航向角
    plt.figure(figsize=(8, 2))
    plt.plot(s, phi, color='blue')
    plt.grid(alpha=0.4, linestyle='--', color='gray')
    plt.xlabel('弧长$\,$(m)', fontsize=18)
    plt.ylabel('航向角$\,$($^\circ$)', fontsize=18)
    plt.xticks(fontsize=18)
    plt.yticks(fontsize=18)
    svg = data_folder + params['file_prefix'] + '_yaw.svg'
    plt.savefig(svg)
    # 曲率
    plt.figure(figsize=(8, 2))
    kappa = datas['kappa']
    kappa_max = (np.ones(len(s))*0.2).tolist()
    plt.plot(s, kappa_max, color='red', linestyle='--', linewidth=3)
    plt.plot(s, kappa, color='blue')
    plt.grid(alpha=0.4, linestyle='--', color='gray')
    plt.xlabel('弧长$\,$(m)', fontsize=18)
    plt.ylabel('曲率$\,$($m^{-1}$)', fontsize=18)
    # plt.legend(['右倾侧翻约束激活阈值', '左倾侧翻约束激活阈值', '滚转角', '俯仰角', '航向角'], fontsize=18)
    plt.xticks(fontsize=18)
    plt.yticks(fontsize=18)
    svg = data_folder + params['file_prefix'] + '_curvature.svg'
    plt.savefig(svg)

def plotSpeedInMultiFiles(folder):
    files = os.listdir(folder)
    for file in files:
        datas = pd.read_csv(folder + file)
        s = datas['s']
        vel = datas['vel']
        plt.plot(s, vel)
    plt.show()


######### read data from file #########
with open(basic_dir+'/params.json') as f:
    params = json.load(f)
speed_file = params['file_prefix'] + '_optimized_speed.csv'
speed_datas = pd.read_csv(data_folder + speed_file)
plotIterationResult(params, speed_datas)
plotOptimizedResult(params, speed_datas)
path_file = params['file_prefix'] + '_path.csv'
path_datas = pd.read_csv(data_folder + path_file)
plotPathGeometry(params, path_datas)
# plotSpeedInMultiFiles(basic_dir + '/../data/' +params['file_prefix'])
plt.show()
