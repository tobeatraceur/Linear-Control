classdef Car

    properties
        bluetooth;
        angleFixNum = 50;
        MAX_SPEED = 50;
        SPEED_FIX = 12.7;
    end

    methods
        function obj = Car(name, channel)
            % Class constructor
            obj.bluetooth = Bluetooth(name, channel);
            fopen(obj.bluetooth);
        end

		function obj = set_MAX_SPEED(obj, newMAX_SPEED)
			%set the max speed for the snake
			obj.MAX_SPEED = newMAX_SPEED;
		end
		
        function thetaFix = get_angle(obj, vicon, nameString)
            obj.set_speed([80, 80]);

            for i = 1: obj.angleFixNum
                vicon.read_data();

                trans = vicon.get_translation(nameString);
                test_x(i) = trans(1);
                test_y(i) = trans(2);

            end
            lastAngle = vicon.get_rotation(nameString);
            obj.stop();

            kandb = polyfit(test_x(2:end), test_y(2:end), 1);
            thetaFix = atan2(kandb(1) * (test_x(end) - test_x(1)), ...
                test_x(end) - test_x(1));
            thetaFix = thetaFix - lastAngle;

        end 

        function command = set_speed(obj, speed)
            speed = speed / obj.SPEED_FIX;
            if(sum(speed.^2) > 2 * obj.MAX_SPEED^2)
                disp('speed too high!!');
                speed = speed * sqrt (2 * obj.MAX_SPEED^2 / sum(speed.^2));
            end
            command = sprintf('{333%+03d%+03d}', round(speed));
            % disp(['command: ', command]);
            fprintf(obj.bluetooth, command);
        end
        
        function stop(obj)
            obj.set_speed([0, 0]);
        end

        function delete(obj)
            fclose(obj.bluetooth);
            delete(obj.bluetooth);
            clear obj.bluetooth;
        end
    end

end
