%% Initiation
% Include necessary files
addpath(genpath('./include/'));

% Start Vicon client
client = VData();

% Initiate a car
car0 = Car('Bt04-A', 1);

% Open file
filename = 'testfile';
outputFile = fopen(['../ViconData/', filename, '.txt'], 'w');

% Log experiment time
fprintf(outputFile, 'Log time: %s\n\n', char(datetime()));


%% Control
% Cast first few frames
for i = 1:100
    data = client.read_data();
    trans = data('GlobalTranslation').Translation;
end

dt=1e-2;    % ����������
T=10;   % ��������� 0 �� T
t=0:dt:T;  % �������ɢʱ������
K_w = 3;
K_1 = 0.0000001;%���Ե�Сһ�㣬����һ��
K_2 = 3;
theta0 = data('GlobalEuler').Rotation(3);%��ʼ�Ƕ�
x0 = trans(1);
y0 = trans(2);
x_T = 1000;%Ŀ��xֵ
y_T = -1000;%Ŀ��yֵ
L = 75.5; %���ּ���һ��

% ���濪ʼ
for k=1:length(t)

    % Read data
    data = client.read_data();
    trans = data('GlobalTranslation').Translation;
    x(k) = trans(1) - x0;
    y(k) = trans(2) - y0;
    theta(k) = data('GlobalEuler').Rotation(3) - theta0;

    v_d = 0.02 * sqrt((x(k)-x_T)^2 + (y(k)-y_T)^2);
    theta_dtheta = atan2((y_T-y(k)), (x_T-x(k)));
    w_d = K_w * (2*pi*round((theta(k)-theta_dtheta)/(2*pi))-theta(k)+theta_dtheta);
    x_e = cos(theta(k)*(x_T-x(k))) + sin(theta(k)*(y_T-y(k)));
    y_e = -sin(theta(k)*(x_T-x(k))) + cos(theta(k)*(y_T-y(k)));
    theta_e = theta_dtheta - theta(k);
    v = v_d * cos(theta_e) + K_1 * x_e;
    w = w_d + v_d * y_e + K_2 * sin(theta(k));

    
    %��������ٶ�
    v_1 = v + L*w;
    v_2 = v - L*w;

    command = car0.set_speed([v_1, v_2] / 25);
    
    % Write data to file
    fprintf(outputFile, '\ntime: %.6f\n', now*60*60*24);
    fprintf(outputFile, 'translation: %10.6f %10.6f %10.6f\n', ...
        trans(1), trans(2), trans(3));
    fprintf(outputFile, 'rotation: %f\n', data('GlobalEuler').Rotation(3));
    fprintf(outputFile, 'velocity: %d %d\n', v_1, v_2);
    fprintf(outputFile, 'command: %s\n', command);
    
end

disp('End control')

% Close client, Bluetooth and output file
% client.close_client();
delete(car0);
fclose(outputFile);
