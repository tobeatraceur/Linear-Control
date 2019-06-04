%% Initiation
% Include necessary files
addpath(genpath('./include/'));

% Start Vicon client
vicon = VData();
disp('Vicon online.');

% Open file
filename = 'data01';
dateString = strsplit(char(datetime));
outputFile = fopen(['../ViconData/19-05-29/', filename, '.txt'], 'w');

% Log experiment time
fprintf(outputFile, 'Log time: %s\n\n', char(datetime()));

% Initialize cars

try
    car0.stop();
    car1.stop();
    car2.stop();
    disp('Cars are already connected.')

catch
    disp('Initializing cars.')

    car0 = Car('btspp://AB5BC3563402', 1);
    disp('Connect to car0')
    car1 = Car('btspp://AB5EC0563402', 1);
    disp('Connect to car1')
    car2 = Car('btspp://ABAFC2563402', 1);
    disp('Connect to car2')

    % Initialize angle 
    test_point_num = 50;

    car0.set_speed([5, 5]);
    car1.set_speed([5, 5]);
    car2.set_speed([5, 5]);

    for i = 1: test_point_num
        vicon.read_data();

        trans = vicon.get_translation('car0');
        test_x0(i) = trans0(1);
        test_y0(i) = trans0(2);

        trans = vicon.get_translation('car1');
        test_x1(i) = trans1(1);
        test_y1(i) = trans1(2);
        
        trans = vicon.get_translation('car2');
        test_x2(i) = trans2(1);
        test_y2(i) = trans2(2);

    end

    kandb = polyfit(test_x0(2:test_point_num), test_y0(2:test_point_num), 1);
    thetaFixCar0 = atan2(kandb(1) * (test_x0(test_point_num) - test_x0(1)), ...
        test_x0(test_point_num) - test_x0(1));

    kandb = polyfit(test_x1(2:test_point_num), test_y1(2:test_point_num), 1);
    thetaFixCar1 = atan2(kandb(1) * (test_x1(test_point_num) - test_x1(1)), ...
        test_x1(test_point_num) - test_x1(1));

    kandb = polyfit(test_x2(2:test_point_num), test_y2(2:test_point_num), 1);
    thetaFixCar2 = atan2(kandb(1) * (test_x2(test_point_num) - test_x2(1)), ...
        test_x2(test_point_num) - test_x2(1));

    vicon.read_data();
    thetaFixCar0 = thetaFixCar0 - vicon.get_rotation('car0');
    thetaFixCar1 = thetaFixCar1 - vicon.get_rotation('car1');
    thetaFixCar2 = thetaFixCar2 - vicon.get_rotation('car2');

    clear kandb, test_x0, test_y0, test_x1, test_y1, test_x2, test_y2, trans

    disp('Angle fix complete.')

end

fprintf(outputFile, 'Theta fix car0: %f\n', thetaFixCar0);
fprintf(outputFile, 'Theta fix car1: %f\n', thetaFixCar1);
fprintf(outputFile, 'Theta fix car2: %f\n', thetaFixCar2);
fprintf(outputFile, '\n\n');


%% Control
% Initialize contoller
controllerCar1 = Controller(thetaFixCar1);
controllerCar2 = Controller(thetaFixCar2);

% A dialog to stop the loop
MessageBox = msgbox( 'Stop demo');

while ishandle( MessageBox )
    % Read data
	tic;
    vicon.read_data();
    vicon.update_trajectory();

    [vl1, vr1, controllerCar1] = controllerCar1.update( ...
        vicon.get_translation('car1'), ...
        vicon.get_rotation('car1'), ...
        vicon.get_translation('car0') + [250; 250; 0], ...
        vicon.get_rotation('car0') + thetaFixCar0, ...
        vicon.get_obstacles('car1'));

    [vl2, vr2, controllerCar2] = controllerCar2.update(...
        vicon.get_translation('car2'), ...
        vicon.get_rotation('car2'), ...
        vicon.get_translation('car0') + [250; -250; 0], ...
        vicon.get_rotation('car0') + thetaFixCar0, ...
        vicon.get_obstacles('car2'));

    commandCar1 = car1.set_speed([vl1, vr1]);
    commandCar2 = car2.set_speed([vl2, vr2]);

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
fclose(outputFile);
car1.stop();car2.stop();
pause(1);
car1.stop();car2.stop();

