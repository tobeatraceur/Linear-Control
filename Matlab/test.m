addpath(genpath('./include/'));

 car0 = Car('btspp://AB5BC3563402', 1);
 disp('Connect to car0')

vicon = VData();
disp('Vicon online.');

car0.set_speed([5 5]);

test_point_num = 50;
speedFix = 12.7;

for i = 1: test_point_num
    vicon.read_data();
    trans0 = vicon.get_translation('car0');
    test_x0(i) = trans0(1);
    test_y0(i) = trans0(2);
    
end

kandb = polyfit(test_x0(2:test_point_num), test_y0(2:test_point_num), 1);
theta_bCar0 = atan2(kandb(1) * (test_x0(test_point_num) - test_x0(1)), ...
    test_x0(test_point_num) - test_x0(1));

car0.set_speed([0 0]);

vicon.read_data();
start_point = vicon.get_translation('car0');

run_time = 100;
step = 30;

for i = 1:run_time/2
    x_follow(i) = start_point(1) + i*step*sin(i*2*pi/run_time*2);
    y_follow(i) = start_point(2) + i*step;
end

for i = run_time/2:run_time
    x_follow(i) = start_point(1) + i*step*sin(i*2*pi/run_time*2);
    y_follow(i) = start_point(2) + (100-i)*step;
end

controllerCar0 = Controller(theta_bCar0 - vicon.get_rotation('car0'));

for i = 1:run_time
    vicon.read_data();
    [vl0, vr0, controllerCar1] = controllerCar0.update(vicon.get_translation('car0'), ...
        vicon.get_rotation('car0'), [x_follow(i);y_follow(i);start_point(3)], ...
        vicon.get_obstacles('car0'));
    
    commandCar0 = car0.set_speed([vl0, vr0] / speedFix);
    pause(0.09)

end

car0.stop();

