dt=1e-1;    % 仿真采样间隔
T=20;   % 仿真区间从 0 到 T
t=0:dt:T;  % 计算的离散时刻序列
K_w = 3;
K_1 = 0.5;%可以调小一点，会稳一点
K_2 = 0.3;
K_4 = 0.0001;
K_trans = 12.7;
K_r = 1000000;
x(1) = 1000;%初始x值
y(1) = 1000;%初始y轴坐标
theta(1) = 0;%初始角度
x_T = 0;%目标x值
y_T = 0;%目标y值
theta_T = 0;%目标角度，没什么用暂时
L = 75.5; %两轮间距的一半

% global kp repDistance;
% kp = 0.2;
% repDistance = 200;

% 仿真开始
for k=1:length(t)
%     %读入当前角度及位置
%     x(k) = 
%     y(k) = 
%     theta(k) = 
    %读取障碍物的位置
    obstacles = [300 200; 400 400];
    
    Grad = potentialFunc(x(k), y(k), x_T, y_T);
    for i = 1:size(obstacles,1)
       Grad = Grad + K_r * ObstaclePotentialFunc(x(k), y(k), obstacles(i,1), obstacles(i,2));
    end
    
    
   % theta_d = theta_T + pi/2;
    v_d = sqrt(Grad(1)^2 + Grad(2)^2);
    theta_dtheta = atan2(-Grad(2), -Grad(1));
    w_d = K_w * (2*pi*round((theta(k)-theta_dtheta)/(2*pi))-theta(k)+theta_dtheta);
    x_e = cos(theta(k)*(x_T-x(k))) + sin(theta(k)*(y_T-y(k)));
    y_e = -sin(theta(k)*(x_T-x(k))) + cos(theta(k)*(y_T-y(k)));
    theta_e = theta_dtheta - theta(k);
    v = v_d * cos(theta_e) + K_1 * x_e;
    w = w_d + K_4 * v_d * y_e + K_2 * sin(theta(k));
    
    x(k+1) = x(k) + cos(theta(k))*v*dt;
    y(k+1) = y(k) + sin(theta(k))*v*dt;
    theta(k+1) = theta(k) + w*dt;
    
    %输出两轮速度
    v_1 = v + L*w;
    v_2 = v - L*w;
end

plot(x,y);

function Grad = potentialFunc(x, y, x_T, y_T)
    kp = 0.2;
    
    Grad = [2*kp*(x-x_T), 2*kp*(y-y_T)];
end

function Grad = ObstaclePotentialFunc(x, y, x_R, y_R)
    repDistance = 200;
    
    if sqrt((x-x_R)^2 + (y-y_R)^2) > repDistance
        Grad = [0, 0];
    else
       Grad = [((x-x_R)^2 + (y-y_R)^2)^(-3/2)*(x_R-x), ((x-x_R)^2 + (y-y_R)^2)^(-3/2)*(y_R-y)];        
    end
end