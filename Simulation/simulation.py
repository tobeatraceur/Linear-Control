
import numpy as np
import sys
import re
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import pandas as pd

pi = math.pi

#读取鹏鹏

data = open("19-06-05//data12.txt")
lines = data.readlines()

real_x_data = []
real_y_data = []
real_t = []
real_rotation = []
Theta_fix = []
real_v1 = []
real_v2 = []

for line in lines:
    print(line)
    if(re.match('translation:',line)):
        output_data = re.findall(r"-?\d+\.?\d*",line)
        real_x_data.append(float(output_data[0]))
        real_y_data.append(float(output_data[1]))
    if(re.match('time:',line)):
        output_data = re.findall(r"-?\d+\.?\d*", line)
        real_t.append(float(output_data[0]))
    if (re.match('rotation:', line)):
        output_data = re.findall(r"-?\d+\.?\d*", line)
        real_rotation.append(float(output_data[0]))
    if(re.match('Theta fix', line)):
        output_data = re.findall(r"-?\d+\.?\d*", line)
        Theta_fix.append(float(output_data[1]))
    if (re.match('command:', line)):
        output_data = re.findall(r"-?\d+\.?\d*", line)
        real_v1.append(float(output_data[1]))
        real_v2.append(float(output_data[2]))

print(real_t)
real_x_data_car_0 = []
real_y_data_car_0 = []
real_x_data_car_1 = []
real_y_data_car_1 = []
real_x_data_car_2 = []
real_y_data_car_2 = []
real_rotation_car_1 = []
real_v1_car1 = []
real_v1_car2 = []
real_v2_car1 = []
real_v2_car2 = []

for i in range(len(real_x_data)):
    if i % 3 == 0:
        real_x_data_car_0.append(real_x_data[i])
        real_y_data_car_0.append(real_y_data[i])
    if i % 3 == 1:
        real_x_data_car_1.append(real_x_data[i])
        real_y_data_car_1.append(real_y_data[i])
        real_rotation_car_1.append(real_rotation[i])
    if i % 3 == 2:
        real_x_data_car_2.append(real_x_data[i])
        real_y_data_car_2.append(real_y_data[i])

for i in range(len(real_v1)):
    if i % 2 == 0:
        real_v1_car1.append(real_v1[i])
        real_v2_car1.append(real_v2[i])
    if i % 2 == 1:
        real_v1_car2.append(real_v1[i])
        real_v2_car2.append(real_v2[i])

plt.figure(1)
plt.plot(real_x_data_car_0, real_y_data_car_0)
plt.figure(1)
plt.scatter(real_x_data_car_1, real_y_data_car_1)
#plt.plot(real_x_data_car_2, real_y_data_car_2)
#读取坤坤


K_v = 0.7
T_v = 0.3
K_w_theta = pi
K_trans = 12.7
K_r = 6e7
Direction_Strength = 2
K_p = 0.5
repDistance = 300

def potentialFunc(x , y , x_T , y_T , targetTheta):
    grad = [2 * K_p * (
                Direction_Strength * np.sin(targetTheta)**2 * (x - x_T) - (Direction_Strength - 1) * np.sin(targetTheta) * np.cos(targetTheta) * (y - y_T) + np.cos(targetTheta)**2 * (x - x_T)) / Direction_Strength,
            2 * K_p * (Direction_Strength * np.cos(targetTheta)**2 * (y - y_T) - (Direction_Strength - 1) * np.sin(
                targetTheta) * np.cos(targetTheta) * (x - x_T) + np.sin(targetTheta)**2 * (y - y_T)) / Direction_Strength]
    return grad

def ObstaclePotentialFunc(x, y, x_R, y_R):

    if np.sqrt((x - x_R)**2 + (y - y_R)**2) > repDistance:
        Grad = [0, 0]
    else:
        Grad = [((x - x_R)**2 + (y - y_R)**2)**(-3 / 2) * (x_R - x),
                ((x - x_R)**2 + (y - y_R)**2)**(-3 / 2) * (y_R - y)]
    return Grad

x = []
y = []
theta = []
theta.append(real_rotation[1] + Theta_fix[1])

print(Theta_fix)
x_T = real_x_data_car_0[0]
y_T = real_y_data_car_0[0] + 400

x.append(real_x_data_car_1[0])
y.append(real_y_data_car_1[0])

theta_T = real_rotation[0] + Theta_fix[0]

L = 75.5
MAX_SPEED = 50 * K_trans
v1_last = 0
v2_last = 0
v1_e_last = 0
v2_e_last = 0

simulation_v1 = []
simulation_v2 = []


for k in range(len(real_t)-1):

    dt = real_t[k + 1] - real_t[k]

    obstacles = [(x_T, y_T-400), (real_x_data_car_2[0], real_y_data_car_2[0])]

    Grad = potentialFunc(x[k], y[k], x_T, y_T, theta_T)
    print(obstacles[0][0])
    for i in range(len(obstacles)):
        grad_0 = ObstaclePotentialFunc(x[k], y[k], obstacles[i][0], obstacles[i][1])
        Grad = Grad + [K_r*a for a in grad_0]

    #print(Grad)

    v_d = np.sqrt(Grad[0]**2 + Grad[1]**2)
    #v_d = 30 * K_trans
    theta_dtheta = math.atan2(-Grad[1], -Grad[0])
    w_d = K_w_theta * (2 * pi * round((theta[k] - theta_dtheta) / (2 * pi)) - theta[k] + theta_dtheta)
    # theta_e = theta_dtheta - theta(k)

    v = v_d
    w = w_d
    v1_d = (v + L * w)
    v2_d = (v - L * w)

    v1_e = v1_d - v1_last
    v2_e = v2_d - v2_last

    v1 = v1_last + K_v * (v1_e - v1_e_last) + (K_v * dt / T_v) * v1_e
    v2 = v2_last + K_v * (v2_e - v2_e_last) + (K_v * dt / T_v) * v2_e

    v1_e_last = v1_e
    v2_e_last = v2_e
    v1_last = v1
    v2_last = v2

    if (v1**2+v2**2 > 2 * MAX_SPEED**2):

        v_1_new = v1 * np.sqrt(2 * MAX_SPEED **2 / (v1**2+v2**2))
        v_2_new = v2 * np.sqrt(2 * MAX_SPEED **2 / (v1**2+v2**2))
        print(v_1_new)
        v1 = v_1_new
        v2 = v_2_new

    simulation_v1.append(round(v1 / K_trans))
    simulation_v2.append(round(v2 / K_trans))

    v_1_new = round(v1 / K_trans) * K_trans
    v_2_new = round(v2 / K_trans) * K_trans

    v_new = (v_1_new + v_2_new) / 2
    w_new = (v_1_new - v_2_new) / (2 * L)


    x.append(x[k] + np.cos(theta[k]) * v_new * dt)
    y.append(y[k] + np.sin(theta[k]) * v_new * dt)
    theta.append(theta[k] + w_new * dt)

simulation_v1.append(0)
simulation_v2.append(0)

#plt.figure(2)
plt.scatter(x, y)
plt.show()

#保存为CSV文件
save_real_x = real_x_data_car_1
save_real_y = real_y_data_car_1
save_real_theta = [i+Theta_fix[1] for i in real_rotation_car_1]
save_real_v1 = real_v1_car1
save_real_v2 = real_v2_car1
save_t = real_t
print(real_t)

save_simulation_x = x
save_simulation_y = y
save_simulation_theta = theta
save_simulation_v1 = simulation_v1
save_simulation_v2 = simulation_v2

print(len(save_real_v2))
print(len(save_simulation_v1))
#字典中的key值即为csv中列名
dataframe = pd.DataFrame({u'实际坐标x': save_real_x, u'真实坐标y': save_real_y,  u'真实角度': save_real_theta,  u'真实指令v1': save_real_v1,
                          u'真实指令v2': save_real_v2,  u'时间': save_t, '仿真坐标x':save_simulation_x, '仿真坐标y':save_simulation_y, '仿真角度': save_simulation_theta,
                          '仿真指令v1': save_simulation_v2
                          , '仿真指令v2': save_simulation_v1
                          })
print(len(save_simulation_v2))
print(len(save_simulation_v1))
#将DataFrame存储为csv,index表示是否显示行名，default=True
#d
#dataframe.to_csv("fixed_point_with_avoidance_12.csv",index=False,sep=',',encoding="gb2312")


