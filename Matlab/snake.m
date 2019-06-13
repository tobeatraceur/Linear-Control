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
finalPosition = vicon.get_translation(carName{1});
finalTheta = vicon.get_rotation(carName{1}) + carThetaFix(1);
disp('begin');

while(headIndex<=size(carList, 2))
    
    MessageBox = msgbox( 'Stop demo');
    
    while ishandle( MessageBox )
        
        % Read data
        tic;
        vicon.read_data();
        vicon.update_trajectory();
        
        reachTargetList = [];
        
        for i = 1:1:headIndex
            
            if(headIndex == szie(car,2) && i == headIndex)
                targetPosition = finalPosition;
                targetTheta = finalTheta;
                
            else
                targetTheta = vicon.get_rotation(carName{i+1}) + carThetaFix(i+1);
                
                thetaMatrix = [cos(targetTheta), -sin(targetTheta), 0;
                    sin(targetTheta), cos(targetTheta), 0;
                    0, 0, 1];
                targetPosition = vicon.get_translation(carName{i+1}) + thetaMatrix*[-400; 0; 0];
                
            end
            
            [vl, vr, reachTarget, carController(i)] = carController(i).update( ...
                vicon.get_translation(carName{i}), ...
                vicon.get_rotation(carName{i}), ...
                targetPosition, ...
                targetTheta, ...
                vicon.get_obstacles(carName{i}));
            
            if i == headIndex
                carList(i) = carList(i).set_MAX_SPEED(20);
            else
                carList(i) = carList(i).set_MAX_SPEED(40);
            end
            
            carList(i).set_speed([vl, vr]);
            
            reachTargetList = [reachTargetList, reachTarget];
            
        end
        
        runTime = toc;
        if runTime < 0.1
            pause(0.1 - runTime)
        else
            warning('Slow loop!')
        end
        
        % reach target and stop
        if sum(reachTargetList) == headIndex
            disp('all cars get the target');
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

