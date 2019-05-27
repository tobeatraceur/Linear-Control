classdef Controller

    properties
        x;
        y;
        theta;
        thetaFix;
        lastTime;

        K_1 = 3;
        K_2 = 0.2;
        K_4 = 0.001;
        K_w_theta = 5;
		K_w = 0.3;
		T_w = 0.1;
		K_r = 5e8;
		K_v = 0.3;
		T_v = 0.1;
        v_last = 0;
		w_last = 0;
        K_p = 2;
        Repulsive_Distance = 800000;

        L = 75.5;

    end

    methods

        function obj = Controller(fix)
            obj.thetaFix = fix;			
            obj.lastTime = now;
            obj.v_last = 0;
            obj.w_last = 0;
        end

        function [v_1, v_2, obj] = update(obj, trans, rotation, target, obstacles)
            % Argument trans is the object's translation.
            % Argument rotation is the object's rotation angle in x-y plane.
            % Argument target is the target translation for this object.
            % Argument obstacles is an array of translations, each of which 
			% Argument dt is time difference
            % stands for an obstacle.

            obj.x(end + 1) = trans(1);
            obj.y(end + 1) = trans(2);
            x_T = target(1);
            y_T = target(2);

            obj.theta(end + 1) = rotation + obj.thetaFix;
            
            %Obstacles avoidance
            Grad = [2*obj.K_p*(obj.x(end)-x_T), 2*obj.K_p*(obj.y(end)-y_T)];
            for i = 1:size(obstacles,1)
                x_R = obstacles(i,1);
                y_R = obstacles(i,2);
                if sqrt((obj.x(end)-x_R)^2 + (obj.y(end)-y_R)^2) < obj.Repulsive_Distance
                    Grad = Grad + obj.K_r * [((obj.x(end)-x_R)^2 + (obj.y(end)-y_R)^2)^(-3/2)*(x_R-obj.x(end)),...
                        ((obj.x(end)-x_R)^2 + (obj.y(end)-y_R)^2)^(-3/2)*(y_R-obj.y(end))];        
                end
            end

            %v_d = 0.5 * sqrt((obj.x(end) - x_T)^2 + ( obj.y(end) - y_T )^2);
            %theta_dtheta = atan2((y_T - obj.y(end)), (x_T - obj.x(end)));
            v_d = sqrt(Grad(1)^2 + Grad(2)^2);
%             v_d = 300;
            theta_dtheta = atan2(-Grad(2), -Grad(1));
            
            w_d = obj.K_w_theta * (2 * pi * round((obj.theta(end) - theta_dtheta)/(2 * pi)) - obj.theta(end) + theta_dtheta);
            
            
%             x_e = cos(obj.theta(end) * (x_T - obj.x(end))) + sin(obj.theta(end) * (y_T - obj.y(end)));
%             y_e = - sin(obj.theta(end) * (x_T - obj.x(end))) + cos(obj.theta(end) * (y_T - obj.y(end)));
%             theta_e = theta_dtheta - obj.theta(end);
%             v = v_d * cos(theta_e) + obj.K_1 * x_e;
%             w = w_d + obj.K_4 * v_d * y_e + obj.K_2 * sin(obj.theta(end));

            dt = (now - obj.lastTime) * 60 * 60 * 24;
            dt = 0.1;
            obj.lastTime = now;
			v = dt/(obj.T_v+dt) * (obj.K_v * v_d + (obj.T_v - obj.K_v*dt)/dt * obj.v_last)
			w = dt/(obj.T_w+dt) * (obj.K_w * w_d + (obj.T_w - obj.K_w*dt)/dt * obj.w_last)
			
			obj.v_last = v;
			obj.w_last = w;
%             v = v_d
%             w = w_d

            v_1 = v - obj.L * w;
            v_2 = v + obj.L * w;

        end

    end

end
