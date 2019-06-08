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
controllerCar1 = Controller(thetaFixCar1);
controllerCar2 = Controller(thetaFixCar2);

%the target, the head, the obstacles
head = controllerCar0;
headCar = car0;
headName = 'car0';
headThetaFix = thetaFixCar0;

follower = [];
followerCar = [];
followerName = {};
followerThetaFix = [];

obstacles = [controllerCar1, controllerCar2];
obstaclesName = {'car1', 'car2'};
obstaclesCar = [car1, car2];
obstaclesThetaFix = [thetaFixCar1, thetaFixCar2];

target = obstacles(1);
targetName = 'car1';
targetCar = obstaclesCar(1);
targetThetaFix = obstaclesThetaFix(1);

% A dialog to stop the loop
MessageBox = msgbox( 'Stop demo');

while ishandle( MessageBox )
    for j=1:1:2
        
        while(1)
            
            % Read data
            tic;
            vicon.read_data();
            vicon.update_trajectory();
            
            %head
            [vlHead, vrHead, reachTarget, head] = head.update( ...
                vicon.get_translation(headName), ...
                vicon.get_rotation(headName), ...
                vicon.get_translation(targetName) + [350; 0; 0], ...
                vicon.get_rotation(targetName) + headThetaFix, ...
                vicon.get_obstacles(headName));
            
            headCar.set_speed([vlHead, vrHead]);
            %follower
            for i = 1:size(follower, 2)
                if i==1
                    [vl, vr, reachTarget_f, follower(i)] = follower(i).update( ...
                        vicon.get_translation(followerName), ...
                        vicon.get_rotation(followerName), ...
                        vicon.get_translation(headName) + [350; 0; 0], ...
                        vicon.get_rotation(headName) + followerThetaFix(i), ...
                        vicon.get_obstacles(followerName));
                    disp('foller');
                else
                    [vl, vr, reachTarget_f, follower(i)] = follower(i).update( ...
                        vicon.get_translation(followerName{i}), ...
                        vicon.get_rotation(followerName(i)), ...
                        vicon.get_translation(followerName{1}) + [350; 0; 0], ...
                        vicon.get_rotation(followerName{i-1}) + followerThetaFix(i), ...
                        vicon.get_obstacles(followerName{i}));
                end
                followerCar(i).set_speed([vl, vr]);
                
            end
            runTime = toc;
            if runTime < 0.1
                pause(0.1 - runTime)
            else
                warning('Slow loop!')
            end
            if reachTarget
                disp('1 end');
                break;
            end
        end
        
        follower = [follower, head];
        followerCar = [followerCar, headCar];
        followerName = 'car0';
        followerThetaFix = [followerThetaFix, headThetaFix];
        
        head = target;
        headCar = targetCar;
        headName = targetName;
        headThetaFix = targetThetaFix;
        
        target = controllerCar2;
        targetName = 'car2';
        targetCar = car2;
        targetThetaFix = thetaFixCar2;
        
    end
    
    
end

disp('End control')

%% Clean
% vicon.close_client();
close all;
car1.stop();car2.stop();
pause(1);
car1.stop();car2.stop();

