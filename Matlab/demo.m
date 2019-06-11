%% Initiation
% Include necessary files
addpath(genpath('./include/'));

% Start Vicon client
vicon = VData();
disp('Vicon online.');

% Open file
filename = 'data23';
dateString = strsplit(char(datetime));
outputFile = fopen(['../ViconData/19-06-05/', filename, '.txt'], 'w');

% Log experiment time
fprintf(outputFile, 'Log time: %s\n\n', char(datetime()));

% Initialize cars
try
    car0.stop();
catch
    fprintf('Connecting to car0... ');
    car0 = Car('btspp://AB5BC3563402', 1);
    thetaFixCar0 = car0.get_angle(vicon, 'car0');
    fprintf('Done\n');
end

try
    car1.stop();
catch
    fprintf('Connecting to car1... ');
    car1 = Car('btspp://AB5EC0563402', 1);
    thetaFixCar1 = car1.get_angle(vicon, 'car1');
    fprintf('Done\n');
end
    
try
    car2.stop();
catch
    fprintf('Connecting to car2... ');
    car2 = Car('btspp://ABAFC2563402', 1);
    thetaFixCar2 = car2.get_angle(vicon, 'car2');
    fprintf('Done\n');
end

fprintf(outputFile, 'Theta fix car0: %f\n', thetaFixCar0);
fprintf(outputFile, 'Theta fix car1: %f\n', thetaFixCar1);
fprintf(outputFile, 'Theta fix car2: %f\n', thetaFixCar2);
fprintf(outputFile, '\n\n');


%% Generate path
vicon.read_data();
start_point = vicon.get_translation('car0');
path_y = 0:15:2000;
path_x = 800 * sin(path_y(:) / 2000 * 2 * pi);
path_y = path_y + start_point(2);
path_x = path_x + start_point(1);
index = 1;


%% Control
% Initialize contoller
controllerCar0 = Controller(thetaFixCar0);
controllerCar1 = Controller(thetaFixCar1);
controllerCar2 = Controller(thetaFixCar2);

% A dialog to stop the loop
MessageBox = msgbox( 'Stop demo');

while ishandle( MessageBox )
    % Read data
	tic;
    vicon.read_data();
    vicon.update_trajectory();

    if index > length(path_x)
        index = length(path_x);
    end

    [path_x(index), path_y(index), 0];
    [vl0, vr0, reach_target, controllerCar0] = controllerCar0.update( ...
        vicon.get_translation('car0'), ...
        vicon.get_rotation('car0'), ...
        [path_x(index), path_y(index), 0] , ...
        vicon.get_rotation('car0') + thetaFixCar0, ...
        []);
    index = index + 1;

    [vl1, vr1, reach_target, controllerCar1] = controllerCar1.update( ...
        vicon.get_translation('car1'), ...
        vicon.get_rotation('car1'), ...
        vicon.get_translation('car0') + [-200; -250; 0], ...
        vicon.get_rotation('car0') + thetaFixCar0, ...
        vicon.get_obstacles('car1'));

    [vl2, vr2, reach_target, controllerCar2] = controllerCar2.update(...
        vicon.get_translation('car2'), ...
        vicon.get_rotation('car2'), ...
        vicon.get_translation('car0') + [200; -250; 0], ...
        vicon.get_rotation('car0') + thetaFixCar0, ...
        vicon.get_obstacles('car2'));

%    commandCar0 = car0.set_speed([vl0, vr0]);
    commandCar1 = car1.set_speed([vl1, vr1]);
    commandCar2 = car2.set_speed([vl2, vr2]);

    % Write data to file
    fprintf(outputFile, 'time: %.6f\n', now*60*60*24);    
    
    fprintf(outputFile, '***** Car0 *****\n');
    fprintf(outputFile, 'translation: %10.6f %10.6f %10.6f\n', ...
        vicon.get_translation('car0'));
    fprintf(outputFile, 'rotation: %f\n', vicon.get_rotation('car0'));
    fprintf(outputFile, 'velocity: %d %d\n', vl0, vr0);
    fprintf(outputFile, 'command: %s\n', commandCar0);
    
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
    
	runTime = toc;
	if runTime < 0.1
		pause(0.1 - runTime)
	else
		warning('Slow loop!')
	end
end

disp('End control')

%% Clean
% vicon.close_client();
close all;
fclose(outputFile);
car0.stop();car1.stop();car2.stop();
pause(1);
car0.stop();car1.stop();car2.stop();

