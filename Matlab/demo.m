%% Initiation
% Include necessary files
addpath(genpath('./include/'));

% Start Vicon client
client = VData();
disp('Vicon online.');

% Initialize a car
car1 = Car('btspp://AB5EC0563402', 1);

% Open file
filename = 'testfile2';
dateString = strsplit(char(datetime));
outputFile = fopen(['../ViconData/', filename, '.txt'], 'w');

% Log experiment time
fprintf(outputFile, 'Log time: %s\n\n', char(datetime()));


%% Control
dt = 1e-1;    % 仿真采样间隔
T = 400;   % 仿真区间从 0 到 T
t = 0 : dt : T;  % 计算的离散时刻序列

k_trans = 12.7;

% Initialize angle
all_data = client.read_data();
car1Data = all_data('car1');
theta0Car1 = car1Data('GlobalEuler').Rotation(3);

test_point_num = 50;
car1.set_speed([5, 5]);
for i = 1: test_point_num
    all_data = client.read_data();
    car1Data = all_data('car1');
    trans = car1Data('GlobalTranslation').Translation;
    test_x(i) = trans(1);
    test_y(i) = trans(2);
end
kandb = polyfit(test_x(2:test_point_num), test_y(2:test_point_num), 1);
theta_bCar1 = atan2(kandb(1) * (test_x(test_point_num) - test_x(1)), ...
    test_x(test_point_num) - test_x(1));

% Initialize contoller
controllerCar1 = Controller(theta_bCar1 - theta0Car1);

for k = 1 : length(t)
    % Read data
    all_data = client.read_data();
    car1Data = all_data('car1');
    car2Data = all_data('car2');

    transCar1 = car1Data('GlobalTranslation').Translation;
    targetTransCar1 = car2Data('GlobalTranslation').Translation;
    rotationCar1 = car1Data('GlobalEuler').Rotation;

    v_1, v_2 = controllerCar1.update(transCar1, rotationCar1, targetTransCar1);

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

% Close client, Bluetooth and output file
% client.close_client();
fclose(outputFile);
car1.stop();
pause(0.5);
car1.stop();
pause(5);
delete(car1);
