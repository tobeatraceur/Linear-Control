close all;
%% Initiation
% Include necessary files
addpath(genpath('./include/'));

% Start Vicon client
vicon = VData();
disp('Vicon online.');

% Open file
filename = 'mouse01';
dateString = strsplit(char(datetime));
outputFile = fopen(['../ViconData/19-06-14/', filename, '.txt'], 'w');

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


%% Control
% Initialize contoller
controllerCar0 = Controller(thetaFixCar0);
controllerCar1 = Controller(thetaFixCar1);
controllerCar2 = Controller(thetaFixCar2);

% A dialog to stop the loop
MessageBox = msgbox( 'Stop demo');

while ishandle( MessageBox ) && ishandle(vicon.fig)
    % Read data
	tic;
    vicon.read_data();
    vicon.update_trajectory();

    % mouse track
    targetPos = vicon.get_mouse();
    if ~vicon.readMouse        
        targetPos = vicon.get_translation('car0')';
    end

    [vl0, vr0, ~, controllerCar0] = controllerCar0.update( ...
        vicon.get_translation('car0'), ...
        vicon.get_rotation('car0'), ...
        targetPos , ...
        vicon.get_rotation('car1') + thetaFixCar1, ...
        vicon.get_obstacles('car0'));

    [vl1, vr1, ~, controllerCar1] = controllerCar1.update( ...
        vicon.get_translation('car1'), ...
        vicon.get_rotation('car1'), ...
        vicon.get_translation('car0') + [-200; -250; 0], ...
        vicon.get_rotation('car0') + thetaFixCar0, ...
        vicon.get_obstacles('none'));

    [vl2, vr2, ~, controllerCar2] = controllerCar2.update(...
        vicon.get_translation('car2'), ...
        vicon.get_rotation('car2'), ...
        vicon.get_translation('car0') + [200; -250; 0], ...
        vicon.get_rotation('car0') + thetaFixCar0, ...
        vicon.get_obstacles('none'));

    commandCar0 = car0.set_speed([vl0, vr0]);
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
try
    close( MessageBox )
end
fclose(outputFile);
car0.stop();car1.stop();car2.stop();
pause(1);
car0.stop();car1.stop();car2.stop();

