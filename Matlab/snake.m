%% Initiation
% Include necessary files
addpath(genpath('./include/'));

% Start Vicon client
vicon = VData();
disp('Vicon online.');
pause(20);
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
vicon.read_data();
filterAlpha = 0.4;
headIndex = 1;
finalPosition = vicon.get_translation(carName{1});
finalTheta = vicon.get_rotation(carName{1}) + carThetaFix(1);
disp('begin');

vicon.read_data();
carLastTheta = [thetaFixCar0 + carThetaFix(2), thetaFixCar1 + carThetaFix(3), thetaFixCar2 + carThetaFix(1)];
finalPosition = vicon.get_translation(carName{1});
finalTheta = vicon.get_rotation(carName{1}) + carThetaFix(1);
%%
disp('begin');
MessageBox = msgbox( 'Stop demo');

while(headIndex<=size(carList, 2))
    % while ishandle( MessageBox )
    
    while ishandle( MessageBox ) && ishandle(vicon.fig)
        
        % Read data
        tic;
        vicon.read_data();
        vicon.update_trajectory();
        
        reachTargetList = [];
        
        for i = 1:1:headIndex
            
            if(headIndex == size(carList,2) && i == headIndex)
                targetPosition = finalPosition;
                targetTheta = finalTheta;
                
            else
                
                targetThetaDesire = vicon.get_rotation(carName{i+1}) + carThetaFix(i+1);
                thetaDelta = round((carLastTheta(i) - targetThetaDesire) / (2 * pi)) * 2 * pi + targetThetaDesire - carLastTheta(i);
                targetTheta = filterAlpha * thetaDelta + carLastTheta(i);
                %                 targetTheta = targetThetaDesire;
                thetaMatrix = [cos(targetTheta), -sin(targetTheta), 0;
                    sin(targetTheta), cos(targetTheta), 0;
                    0, 0, 1];
                targetPosition = vicon.get_translation(carName{i+1}) + thetaMatrix*[-300; 150; 0]
                
            end
            
            carLastTheta(i) = targetTheta;
            
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
%             if reachTarget
%                 vl = 0;
%                 vr = 0;
%             end
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
        if reachTargetList(headIndex) == true && headIndex ~= size(carList, 2)
            disp('head get the target!');
            break;
        end
        
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

car0.set_MAX_SPEED(30);car1.set_MAX_SPEED(30);car2.set_MAX_SPEED(30);

car0.stop();car1.stop();car2.stop();
pause(1);
car0.stop();car1.stop();car2.stop();


%%
%turn to triangle
while ishandle( MessageBox )
    % Read data
    tic;
    vicon.read_data();
    vicon.update_trajectory();
    
    targetPos = vicon.get_translation('car2')';
    [vl2, vr2, reachTarget, controllerCar2] = controllerCar2.update( ...
        vicon.get_translation('car2'), ...
        vicon.get_rotation('car2'), ...
        targetPos , ...
        vicon.get_rotation('car2') + thetaFixCar2, ...
        vicon.get_obstacles('car2'));
    
    targetPosition = vicon.get_translation('car2') + [-400; -450; 0];
    
    [vl1, vr1, reachTarget1, controllerCar1] = controllerCar1.update( ...
        vicon.get_translation('car1'), ...
        vicon.get_rotation('car1'), ...
        targetPosition, ...
        vicon.get_rotation('car2') + carThetaFix(3), ...
        vicon.get_obstacles('car1'));
    
    if reachTarget1
        v1l = 0;
        vr1 = 0;
        disp('car1 reach!');
    end
    
    targetPosition = vicon.get_translation('car2') + [400; -450; 0];
    
    [vl0, vr0, reachTarget0, controllerCar0] = controllerCar0.update(...
        vicon.get_translation('car0'), ...
        vicon.get_rotation('car0'), ...
        targetPosition, ...
        vicon.get_rotation('car2') + carThetaFix(3), ...
        vicon.get_obstacles('car0'));
    if reachTarget0
        vl0 = 0;
        vr0 = 0;
        disp('car0 reach!');
    end
    
    if reachTarget1 && reachTarget0
        disp('turn o triangle');
        break;
    end
    
    
    commandCar0 = car0.set_speed([vl0, vr0]);
    commandCar1 = car1.set_speed([vl1, vr1]);
    commandCar2 = car2.set_speed([vl2, vr2]);
    
    runTime = toc;
    if runTime < 0.1
        pause(0.1 - runTime)
    else
        warning('Slow loop!')
    end
    
end

car0.stop();car1.stop();car2.stop();
pause(1);
car0.stop();car1.stop();car2.stop();

disp('please click');

%% Generate path
for i = 1:100
    vicon.read_data();
end
start_point = vicon.get_translation('car2');
path_y = 0:15:3000;
path_x = 600 * sin(path_y(:) / 2000 * 2 * pi);
path_y = path_y + start_point(2);
path_x = path_x + start_point(1);
index = 1;

car0.set_MAX_SPEED(50);car1.set_MAX_SPEED(50);car2.set_MAX_SPEED(50);

while ishandle( MessageBox ) && ishandle(vicon.fig)
    % Read data
    tic;
    vicon.read_data();
    vicon.update_trajectory();
    
    if index > length(path_x)
        index = length(path_x);
    end
    
    targetPos = vicon.get_mouse()
    if ~vicon.readMouse
        targetPos = [path_x(index), path_y(index), 0];
    end
%     targetPos = vicon.get_translation('car2')';
    
    [vl2, vr2, reachTarget, controllerCar2] = controllerCar2.update( ...
        vicon.get_translation('car2'), ...
        vicon.get_rotation('car2'), ...
        targetPos , ...
        vicon.get_rotation('car2') + thetaFixCar2, ...
        vicon.get_obstacles('car2'));
    
    %     if reachTarget
    %         vl2 = 0;
    %         vr2 = 0;
    %     end
    
    targetThetaDesire = vicon.get_rotation('car2') + carThetaFix(3);
    thetaDelta = round((carLastTheta(2) - targetThetaDesire) / (2 * pi)) * 2 * pi + targetThetaDesire - carLastTheta(2)
    targetTheta = filterAlpha * thetaDelta + carLastTheta(2);
    targetTheta = targetThetaDesire;
    thetaMatrix = [cos(targetTheta), -sin(targetTheta), 0;
        sin(targetTheta), cos(targetTheta), 0;
        0, 0, 1];
    targetPosition = vicon.get_translation('car2') + [-400; -450; 0];
    carLastTheta(2) = targetTheta;
    
    [vl1, vr1, reachTarget, controllerCar1] = controllerCar1.update( ...
        vicon.get_translation('car1'), ...
        vicon.get_rotation('car1'), ...
        targetPosition, ...
        targetTheta, ...
        vicon.get_obstacles('car1'));
    
    %     if reachTarget
    %         vl1 = 0;
    %         vr1 = 0;
    %     end
    
    targetThetaDesire = vicon.get_rotation('car2') + carThetaFix(3);
    thetaDelta = round((carLastTheta(1) - targetThetaDesire) / (2 * pi)) * 2 * pi + targetThetaDesire - carLastTheta(1)
    targetTheta = filterAlpha * thetaDelta + carLastTheta(1);
    targetTheta = targetThetaDesire;
    thetaMatrix = [cos(targetTheta), -sin(targetTheta), 0;
        sin(targetTheta), cos(targetTheta), 0;
        0, 0, 1];
    targetPosition = vicon.get_translation('car2') + [400; -450; 0];
    carLastTheta(1) = targetTheta;
    
    [vl0, vr0, reachTarget, controllerCar0] = controllerCar0.update(...
        vicon.get_translation('car0'), ...
        vicon.get_rotation('car0'), ...
        targetPosition, ...
        targetTheta, ...
        vicon.get_obstacles('car0'));
    
    %     if reachTarget
    %         vl0 = 0;
    %         vr0 = 0;
    %     end
    index = index + 1;
    
    commandCar0 = car0.set_speed([vl0, vr0]);
    commandCar1 = car1.set_speed([vl1, vr1]);
    commandCar2 = car2.set_speed([vl2, vr2]);
    
    
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
close all;
car0.stop();car1.stop();car2.stop();
pause(1);
car0.stop();car1.stop();car2.stop();

