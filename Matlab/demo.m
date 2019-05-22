%% Initiation
% Include necessary files
addpath(genpath('./include/'));

% Start Vicon client
client = VData();
disp('Vicon online.');

% Initiate a car
car1 = Car('btspp://AB5EC0563402', 1);

% Open file
filename = 'testfile2';
dateString = strsplit(char(datetime));
outputFile = fopen(['../ViconData/', filename, '.txt'], 'w');

% Log experiment time
fprintf(outputFile, 'Log time: %s\n\n', char(datetime()));


%% Control

% Get first location
all_data = client.read_data();
car1Data = all_data('car1');
car2Data = all_data('car2');
trans = car1Data('GlobalTranslation').Translation;
targetTrans = car2Data('GlobalTranslation').Translation;

dt=1e-1;    % 仿真采样间隔
T=400;   % 仿真区间从 0 到 T
t=0:dt:T;  % 计算的离散时刻序列
K_w = 0.3;
K_1 = 3;
K_2 = 0.3;
K_4 = 0.001;
theta0 = car1Data('GlobalEuler').Rotation(3);%初始角度
x_T = targetTrans(1);
y_T = targetTrans(2);
L = 75.5; %两轮间距的一半
k_trans = 12.7;

% 仿真开始
tic

test_point_num = 50;
for i = 1: test_point_num
    car1.set_speed([5, 5]);
    all_data = client.read_data();
    car1Data = all_data('car1');
    trans = car1Data('GlobalTranslation').Translation;
    test_x(i) = trans(1);
    test_y(i) = trans(2);
end

kandb = polyfit(test_x(2:test_point_num), test_y(2:test_point_num), 1);
theta_b = atan2(kandb(1) * (test_x(test_point_num) - test_x(1)), ...
    test_x(test_point_num) - test_x(1));

for k=1:length(t)
    % Read data
    all_data = client.read_data();
    car1Data = all_data('car1');
    car2Data = all_data('car2');
    trans = car1Data('GlobalTranslation').Translation;
    targetTrans = car2Data('GlobalTranslation').Translation;
    x(k) = trans(1);
    y(k) = trans(2);
    x_T = targetTrans(1);
    y_T = targetTrans(2);
    theta(k) = car1Data('GlobalEuler').Rotation(3) + theta_b - theta0;

    v_d = 0.5 * sqrt((x(k)-x_T)^2 + (y(k)-y_T)^2);
    theta_dtheta = atan2((y_T-y(k)), (x_T-x(k)));
    w_d = K_w * (2*pi*round((theta(k)-theta_dtheta)/(2*pi))-theta(k)+theta_dtheta);
    x_e = cos(theta(k)*(x_T-x(k))) + sin(theta(k)*(y_T-y(k)));
    y_e = -sin(theta(k)*(x_T-x(k))) + cos(theta(k)*(y_T-y(k)));
    theta_e = theta_dtheta - theta(k);
    v = v_d * cos(theta_e) + K_1 * x_e;
    w = w_d + K_4 * v_d * y_e + K_2 * sin(theta(k));

    
    %输出两轮速度
    v_1 = v + L*w;
    v_2 = v - L*w;

    command = car1.set_speed([v_1, v_2] / k_trans);
    
    % Write data to file
    fprintf(outputFile, '\ntime: %.6f\n', now*60*60*24);
    fprintf(outputFile, 'translation: %10.6f %10.6f %10.6f\n', ...
        trans(1), trans(2), trans(3));
    fprintf(outputFile, 'rotation: %f\n', car1Data('GlobalEuler').Rotation(3));
    fprintf(outputFile, 'velocity: %d %d\n', v_1, v_2);
    fprintf(outputFile, 'command: %s\n', command);
    pause(0.09)
end

disp('End control')
toc

% Close client, Bluetooth and output file
% client.close_client();
fclose(outputFile);
car1.stop();
pause(0.5);
car1.stop();
pause(5); % make sure the cars has stopped
delete(car1);
