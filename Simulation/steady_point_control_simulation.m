dt=1e-3;    % 仿真采样间隔
T=10;   % 仿真区间从 0 到 T
t=0:dt:T;  % 计算的离散时刻序列

K_w = 0.3;
K_1 = 0.5;%可以调小一点，会稳一点
K_2 = 0.3;
K_4 = 0.001;

x(1) = 0;%初始x值
y(1) = 0;%初始y轴坐标
theta(1) = 0;%初始角度

x_T = 1000;%目标x值
y_T = 1000;%目标y值
theta_T = 0;%目标角度，没什么用暂时



L = 75.5; %两轮间距的一半

k_trans = 12.7;
num = 1;
% % 仿真开始
    for k=1:length(t)
       % theta_d = theta_T + pi/2;
        v_d = 0.02 * sqrt((x(num)-x_T)^2 + (y(num)-y_T)^2);
        theta_dtheta = atan2((y_T-y( num)), (x_T-x( num)));
        w_d = K_w * (2*pi*round((theta( num)-theta_dtheta)/(2*pi))-theta( num)+theta_dtheta);
        x_e = cos(theta(num)*(x_T-x(num))) + sin(theta(num)*(y_T-y(num)));
        y_e = -sin(theta(num)*(x_T-x( num))) + cos(theta(num)*(y_T-y( num)));
        theta_e = theta_dtheta - theta( num);
        v = v_d * cos(theta_e) + K_1 * x_e;
        w = w_d + K_4 * v_d * y_e + K_2 * sin(theta( num));
        %两轮速度
        v_1 = v + L*w;
        v_2 = v - L*w;

        v_1_new = int8(v_1/k_trans)*k_trans;
        v_2_new = int8(v_2/k_trans)*k_trans;
        
        if v_1_new > 50*k_trans
            v_1_new = 50*k_trans;
        else
            if v_1_new < -50*k_trans
                v_1_new = -50*k_trans;
            end
        end
        
        if v_2_new > 50*k_trans
            v_2_new = 50*k_trans;
        else
            if v_2_new < -50*k_trans
                v_2_new = -50*k_trans;
            end
        end
        v_new = (v_1_new + v_2_new)/2;
        w_new = (v_1_new - v_2_new)/(2*L);
        
        x(num+1) = x(num) + cos(theta(num))*v;
        y(num+1) = y(num) + sin(theta(num))*v;
        theta( num+1) = theta(num) + w;

        num = num + 1;
    end

%subplot(4,1,1)

%plot(x)
%axis([0, 20, 0 ,50])
%subplot(4,1,2)
%plot(y)
%subplot(4,1,3)
%plot(theta)
%subplot(4,1,4)
%plot(y)
figure(1)
scatter(x,y)
figure(2)
plot(x,y)
