dt=1e-2;    % 仿真采样间隔
T=10;   % 仿真区间从 0 到 T
t=0:dt:T;  % 计算的离散时刻序列
K_w = 3;
K_1 = 0.001;%可以调小一点，会稳一点
K_2 = 3;
%x(1) = 0;%初始x值
%y(1) = 0;%初始y轴坐标
theta(1) = 0;%初始角度
x_T = 5;%目标x值
y_T = 5;%目标y值
theta_T = 0;%目标角度，没什么用暂时
L = 75.5; %两轮间距的一半

% 仿真开始
for k=1:length(t)
    %读入当前角度及位置
    x(k) = 
    y(k) = 
    theta(k) = 
    
   % theta_d = theta_T + pi/2;
    v_d = 0.02 * sqrt((x(k)-x_T)^2 + (y(k)-y_T)^2);
    theta_dtheta = atan2((y_T-y(k)), (x_T-x(k)));
    w_d = K_w * (2*pi*round((theta(k)-theta_dtheta)/(2*pi))-theta(k)+theta_dtheta);
    x_e = cos(theta(k)*(x_T-x(k))) + sin(theta(k)*(y_T-y(k)));
    y_e = -sin(theta(k)*(x_T-x(k))) + cos(theta(k)*(y_T-y(k)));
    theta_e = theta_dtheta - theta(k);
    v = v_d * cos(theta_e) + K_1 * x_e;
    w = w_d + v_d * y_e + K_2 * sin(theta(k));
    
    %x(k+1) = x(k) + cos(theta(k))*v;
    %y(k+1) = y(k) + sin(theta(k))*v;
    %theta(k+1) = theta(k) + w;
    
    %输出两轮速度
    v_1 = v + L*w;
    v_2 = v - L*w;
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
