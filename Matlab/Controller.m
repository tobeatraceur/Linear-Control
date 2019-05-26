classdef Controller

    properties
        x;
        y;
        theta;
        thetaFix;

        K_w = 0.3;
        K_1 = 3;
        K_2 = 0.3;
        K_4 = 0.001;

        L = 75.5;

    end

    methods

        function obj = Controller(fix)
            obj.thetaFix = fix;
        end

        function [v_1, v_2] = update(obj, trans, rotation, target, obstacles)
            % Argument trans is the object's translation.
            % Argument rotation is the object's rotation angle in x-y plane.
            % Argument target is the target translation for this object.
            % Argument obstacles is an array of translations, each of which 
            % stands for an obstacle.

            obj.x(end + 1) = trans(1);
            obj.y(end + 1) = trans(2);
            x_T = target(1);
            y_T = target(2);

            obj.theta(end + 1) = rotation + obj.thetaFix;

            v_d = 0.5 * sqrt((obj.x(end) - x_T)^2 + ( obj.y(end) - y_T )^2);
            theta_dtheta = atan2((y_T - obj.y(end)), (x_T - obj.x(end)));
            w_d = obj.K_w * (2 * pi * round((obj.theta(end) - theta_dtheta)/(2 * pi)) - obj.theta(end) + theta_dtheta);
            x_e = cos(obj.theta(end) * (x_T - obj.x(end))) + sin(obj.theta(end) * (y_T - obj.y(end)));
            y_e = - sin(obj.theta(end) * (x_T - obj.x(end))) + cos(obj.theta(end) * (y_T - obj.y(end)));
            theta_e = theta_dtheta - obj.theta(end);
            v = v_d * cos(theta_e) + obj.K_1 * x_e;
            w = w_d + obj.K_4 * v_d * y_e + obj.K_2 * sin(obj.theta(end));

            v_1 = v + obj.L * w;
            v_2 = v - obj.L * w;

        end

    end

end
