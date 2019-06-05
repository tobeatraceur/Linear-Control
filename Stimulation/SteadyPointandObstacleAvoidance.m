clear all;
dt=1e-1;    % 仿真采样间隔
T=20;   % 仿真区间从 0 到 T
t=0:dt:T;  % 计算的离散时刻序列

K_v = 0.7;
T_v = 0.3;
K_w_theta = pi;
K_trans = 12.7;
K_r = 60000000;
%Direction_Strength = 5;
x(1) = 1000;%初始x值
y(1) = 1000;%初始y轴坐标
theta(1) = 0;%初始角度
x_T = 0;%目标x值
y_T = 0;%目标y值
theta_T = 0;%目标角度，没什么用暂时
L = 75.5; %两轮间距的一半
v1_last = 0;
v2_last = 0;
v1_e_last = 0;
v2_e_last = 0;

% 仿真开始
for k=1:length(t)
    %     %读入当前角度及位置
    %     x(k) =
    %     y(k) =
    %     theta(k) =
    %读取障碍物的位置
    obstacles = [500 600; 400 400; 300 400; 800 200];
    Grad = potentialFunc(x(k), y(k), x_T, y_T, theta_T);
    for i = 1:size(obstacles,1)
        Grad = Grad + K_r * ObstaclePotentialFunc(x(k), y(k), obstacles(i,1), obstacles(i,2));
    end
    
    %v_d = sqrt(Grad(1)^2 + Grad(2)^2);
    v_d = 30*K_trans;
    theta_dtheta = atan2(-Grad(2), -Grad(1));
    w_d = K_w_theta * (2*pi*round((theta(k)-theta_dtheta)/(2*pi))-theta(k)+theta_dtheta);
    %theta_e = theta_dtheta - theta(k);
    
    
    v = v_d; w = w_d;
    v1_d = (v + L*w);
    v2_d = (v - L*w);
    
    v1_e = v1_d - v1_last;
    v2_e = v2_d - v2_last;
    
    
    v1 = v1_last + K_v*(v1_e - v1_e_last) + (K_v*dt/T_v)*v1_e;
    v2 = v2_last + K_v*(v2_e - v2_e_last) + (K_v*dt/T_v)*v2_e;
    
    v1_e_last = v1_e;
    v2_e_last = v2_e;
    v1_last = v1;
    v2_last = v2;
    v_1_new = round(v1/K_trans)*K_trans;
    v_2_new = round(v2/K_trans)*K_trans;
    
    v_new = (v_1_new + v_2_new)/2;
    w_new = (v_1_new - v_2_new)/(2*L);
    
    
    x(k+1) = x(k) + cos(theta(k))*v_new*dt;
    y(k+1) = y(k) + sin(theta(k))*v_new*dt;
    theta(k+1) = theta(k) + w_new*dt;
    
end
scatter(x,y);
hold on;
for i = 1:size(obstacles,1)
    scatter(obstacles(i,1),obstacles(i,2));
end
hold off;
function Grad = potentialFunc(x, y, x_T, y_T, targetTheta)
Direction_Strength=5;
K_p = 0.5;

Grad = [2*K_p*(Direction_Strength*sin(targetTheta)^2*(x-x_T)-(Direction_Strength-1)*sin(targetTheta)*cos(targetTheta)*(y-y_T)+cos(targetTheta)^2*(x-x_T))/Direction_Strength,
    2*K_p*(Direction_Strength*cos(targetTheta)^2*(y-y_T)-(Direction_Strength-1)*sin(targetTheta)*cos(targetTheta)*(x-x_T)+sin(targetTheta)^2*(y-y_T))/Direction_Strength];

end

function Grad = ObstaclePotentialFunc(x, y, x_R, y_R)
repDistance = 300;

if sqrt((x-x_R)^2 + (y-y_R)^2) > repDistance
    Grad = [0, 0];
else
    Grad = [((x-x_R)^2 + (y-y_R)^2)^(-3/2)*(x_R-x), ((x-x_R)^2 + (y-y_R)^2)^(-3/2)*(y_R-y)];
end
end