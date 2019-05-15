from scipy.integrate import odeint
import numpy as np
import sys
import re
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression

L_average = 75.67477
u2 = 0
u1 = 0

def dState(x, y, theta):
    dx = ((u1+u2)/2)*np.cos(theta)
    dy = ((u1+u2)/2)*np.sin(theta)
    dtheta = (1/L)*(u2 - u1)


number = [1, 2, 3, 5, 7, 9]
Lower = [4800,2600,1000,1000,200,500]
Upper = [5500,4000,2200,1300,1400,1400]
real_v = []
choice = 0
for readnum in number:
    real_x_data = []
    real_y_data = []
    real_t = []

    data = open("forward"+str(readnum)+"-"+str(readnum)+".txt")
    #data = open("spin-4+9.txt")
    lines = data.readlines()

    for line in lines:
        if(re.match('Global Translation:',line)):
            output_data = re.findall(r"\d+\.?\d*",line)
            real_x_data.append(float(output_data[0]))
            real_y_data.append(float(output_data[1]))
        if(re.match('time:',line)):
            output_data = re.findall(r"\d+\.?\d*", line)
            real_t.append(float(output_data[0]))

    x_data = []
    y_data = []
    t = []
    v = []

    for i in range(Lower[choice],Upper[choice]):
        x_data.append(real_x_data[i])
        y_data.append(real_y_data[i])
        t.append(real_t[i])
        if(i!=0):
            #print(np.sqrt(((real_x_data[i]-real_x_data[i-1])**2)+((real_y_data[i]-real_y_data[i-1])**2)))
            v.append(np.sqrt(((real_x_data[i]-real_x_data[i-1])**2)+((real_y_data[i]-real_y_data[i-1])**2))/(real_t[i]-real_t[i-1]))
    #model = np.polyfit(x_data,y_data,1)
    #print(model)
    #plt.plot(real_x_data, real_y_data,'r')
    #print(v)
    #print(len(lines)/5)
    #print(np.mean(v))
    real_v.append(np.mean(v))
    choice += 1
    #plt.plot(x_data,y_data)
    #plt.show()

#print(real_v)
v_model = np.polyfit(number,real_v,1)
p1 = np.poly1d(v_model)
#print(p1(1))

leftv = ['+2','+4','0','0']
rightv = ['+5','+9','+2','+9']
left = [2,4,0,0]
right = [5,9,2,9]
LO = [320,100,1200,600]
Up = [800,800,1800,800]

for runtime in range(4):

    real_x_data = []
    real_y_data = []
    real_t = []

    data = open("spin"+leftv[runtime]+rightv[runtime]+".txt")
    # data = open("spin-4+9.txt")
    lines = data.readlines()

    for line in lines:
        if (re.match('Global Translation:', line)):
            output_data = re.findall(r"\d+\.?\d*", line)
            real_x_data.append(float(output_data[0]))
            real_y_data.append(float(output_data[1]))
        if (re.match('time:', line)):
            output_data = re.findall(r"\d+\.?\d*", line)
            real_t.append(float(output_data[0]))

    x_data = []
    y_data = []
    x2_y2_data = []
    t = []
    v = []

    #print(len(real_x_data))

    for i in range(LO[runtime], Up[runtime]):
        x_data.append(real_x_data[i])
        y_data.append(real_y_data[i])
        x2_y2_data.append(real_x_data[i]**2+real_y_data[i]**2)
        t.append(real_t[i])
        if (i != 0):
            v.append(
                np.sqrt(((real_x_data[i] - real_x_data[i - 1]) ** 2) + ((real_y_data[i] - real_y_data[i - 1]) ** 2)) / (
                        real_t[i] - real_t[i - 1]))

    y_train = np.array(x2_y2_data).reshape(-1,1)
    X_train = np.hstack((np.array(x_data).reshape(-1,1),np.array(y_data).reshape(-1,1)))
    linreg = LinearRegression()
    model= linreg.fit(X_train, y_train)

    A = linreg.coef_[0,0]
    B = linreg.coef_[0,1]
    C = linreg.intercept_[0]

    x0 = A/2
    y0 = B/2
    R = np.sqrt(C+x0**2+y0**2)
    #print(R)

    v0 = np.mean(v)
    v1 = p1(left[runtime])
    v2 = p1(right[runtime])

    L = R*(v2-v1)/(2*v0)
    print(L)

    theta = np.arange(0, 2 * np.pi, 0.01)
    x = -(v1/v0)*R + R * np.cos(theta)
    y = R * np.sin(theta)

    plt.plot(x, y)
    #plt.plot(real_x_data, real_y_data,'r')
    #plt.plot(x_data,y_data,'g')

plt.show()
