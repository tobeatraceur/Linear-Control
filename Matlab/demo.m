%% Initiation
% Include necessary files
addpath(genpath('./include/'));

% Start Vicon client
vicon = VData();
disp('Vicon online.');

% Initialize a car
car1 = Car('btspp://AB5EC0563402', 1);
disp('Connect to car1')
car2 = Car('btspp://ABAFC2563402', 1);
disp('Connect to car2')

% Open file
filename = 'endGame';
dateString = strsplit(char(datetime));
outputFile = fopen(['../ViconData/', filename, '.txt'], 'w');

% Log experiment time
fprintf(outputFile, 'Log time: %s\n\n', char(datetime()));

%% Control

speedFix = 12.7;

% Initialize angle
test_point_num = 50;

car1.set_speed([5, 5]);
car2.set_speed([5, 5]);

for i = 1: test_point_num
    vicon.read_data();
    trans1 = vicon.get_translation('car1');
    test_x1(i) = trans1(1);
    test_y1(i) = trans1(2);
    
    trans2 = vicon.get_translation('car2');
    test_x2(i) = trans2(1);
    test_y2(i) = trans2(2);

    fprintf(outputFile, 'car1 translation: %10.6f %10.6f %10.6f\n', ...
        vicon.get_translation('car1'));
    fprintf(outputFile, 'car2 translation: %10.6f %10.6f %10.6f\n', ...
        vicon.get_translation('car2'));
end
% 
kandb = polyfit(test_x1(2:test_point_num), test_y1(2:test_point_num), 1);
theta_bCar1 = atan2(kandb(1) * (test_x1(test_point_num) - test_x1(1)), ...
    test_x1(test_point_num) - test_x1(1));

kandb = polyfit(test_x2(2:test_point_num), test_y2(2:test_point_num), 1);
theta_bCar2 = atan2(kandb(1) * (test_x2(test_point_num) - test_x2(1)), ...
    test_x2(test_point_num) - test_x2(1));

% Initialize contoller
controllerCar1 = Controller(theta_bCar1 - vicon.get_rotation('car1'));
controllerCar2 = Controller(theta_bCar2 - vicon.get_rotation('car2'));

% A dialog to stop the loop
MessageBox = msgbox( 'Stop demo');

while ishandle( MessageBox )
    % Read data
    vicon.read_data();

    [vl1, vr1, controllerCar1] = controllerCar1.update(vicon.get_translation('car1'), ...
        vicon.get_rotation('car1'), vicon.get_translation('car0') + [250; 150; 0], ...
        vicon.get_obstacles('car1'));

    [vl2, vr2, controllerCar2] = controllerCar2.update(vicon.get_translation('car2'), ...
        vicon.get_rotation('car2'), vicon.get_translation('car0') + [250; -150; 0], ...
        vicon.get_obstacles('car2'));
    
    commandCar1 = car1.set_speed([vl1, vr1] / speedFix);
    commandCar2 = car2.set_speed([vl2, vr2] / speedFix);
    
    % Write data to file
    fprintf(outputFile, 'time: %.6f\n', now*60*60*24);    
    
    fprintf(outputFile, '***** Car0 *****\n');
    fprintf(outputFile, 'translation: %10.6f %10.6f %10.6f\n', ...
        vicon.get_translation('car0'));
    
    fprintf(outputFile, '***** Car1 *****\n');
    fprintf(outputFile, 'translation: %10.6f %10.6f %10.6f\n', ...
        vicon.get_translation('car1'));
    fprintf(outputFile, 'rotation: %f\n', vicon.get_rotation('car1'));
    fprintf(outputFile, 'velocity: %d %d\n', vl1, vr1);
    fprintf(outputFile, 'command: %s\n', commandCar1);
    
    fprintf(outputFile, '***** Car2 *****\n');
    fprintf(outputFile, 'translation: %10.6f %10.6f %10.6f\n', ...
        vicon.get_translation('car2'));
    fprintf(outputFile, 'rotation: %f\n', vicon.get_rotation('car2'));
    fprintf(outputFile, 'velocity: %d %d\n', vl2, vr2);
    fprintf(outputFile, 'command: %s\n', commandCar2);

    fprintf(outputFile, '\n\n');
    
    pause(0.09)
end

disp('End control')

% Close vicon client, Bluetooth and output file
% vicon.close_client();
fclose(outputFile);
car1.stop();
car2.stop();
pause(0.5);
car1.stop();
car2.stop();
pause(5);
delete(car1);
delete(car2);
