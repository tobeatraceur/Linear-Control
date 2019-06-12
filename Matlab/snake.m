%% Initiation
% Include necessary files
addpath(genpath('./include/'));

% Start Vicon client
vicon = VData();
disp('Vicon online.');

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


%% Control
% Initialize contoller
controllerCar0 = Controller(thetaFixCar0);
controllerCar1 = Controller(thetaFixCar1);
controllerCar2 = Controller(thetaFixCar2);

%the car
carController = [controllerCar0, controllerCar1, controllerCar2];
carList = [car0, car1, car2];
carName = {'car0', 'car1', 'car2'};
carThetaFix = [thetaFixCar0, thetaFixCar1, thetaFixCar2];


%Initialization
headIndex = 1;
disp('begin');

while(headIndex<size(carList, 2))
    
    head = carController(headIndex);
    headCar = carList(headIndex);
    headName = carName{headIndex};
    headThetaFix = carThetaFix(headIndex);
    
    target = carController(headIndex+1);
    targetCar = carList(headIndex+1);
    targetName = carName{headIndex+1};
    targetThetaFix = carThetaFix(headIndex+1);
    
    MessageBox = msgbox( 'Stop demo');
    
    while ishandle( MessageBox )
        
        % Read data
        tic;
        vicon.read_data();
        vicon.update_trajectory();
        
        %head
        relativeTheta = vicon.get_rotation(targetName) + targetThetaFix;
        thetaMatrix = [cos(relativeTheta), sin(relativeTheta), 0;
            -sin(relativeTheta), cos(relativeTheta), 0;
            0, 0, 1];
        vicon.get_translation(targetName) + thetaMatrix*[-600; 0; 0]
        [vlHead, vrHead, reachTarget, head] = head.update( ...
            vicon.get_translation(headName), ...
            vicon.get_rotation(headName), ...
            vicon.get_translation(targetName) + thetaMatrix*[-600; 0; 0], ...
            vicon.get_rotation(targetName) + targetThetaFix, ...
            vicon.get_obstacles(headName));
        
        headCar.set_speed([vlHead, vrHead]/4);
        
        %follower
        
        for i = 1:(headIndex-1)
            relativeTheta = vicon.get_rotation(carName{i+1}) + carThetaFix(i+1);
            thetaMatrix = [cos(relativeTheta), sin(relativeTheta), 0;
                -sin(relativeTheta), cos(relativeTheta), 0;
                0, 0, 1];
            [vl, vr, reachTarget_f, carController(i)] = carController(i).update( ...
                vicon.get_translation(carName{i}), ...
                vicon.get_rotation(carName{i}), ...
                vicon.get_translation(carName{i+1}) + thetaMatrix*[-600; 0; 0], ...
                vicon.get_rotation(carName{i+1}) + carThetaFix(i+1), ...
                vicon.get_obstacles(carName{i}));
            
            carList(i).set_speed([vl, vr]);
            
        end
        
        runTime = toc;
        if runTime < 0.1
            pause(0.1 - runTime)
        else
            warning('Slow loop!')
        end
        % reach target and stop
        if reachTarget
            disp('the head get the target');
            break;
        end
    end
    
    if ~ishandle( MessageBox )
        break;
    end
    
    headIndex = headIndex+1;
end

disp('End control')

%% Clean
% vicon.close_client();
close all;
car0.stop();car1.stop();car2.stop();
pause(1);
car0.stop();car1.stop();car2.stop();

