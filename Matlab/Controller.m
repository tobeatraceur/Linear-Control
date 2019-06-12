classdef Controller
    % TODO: try handler class
    properties
        x;
        y;
        theta;
        thetaFix;
        lastTime;
        v1_last;
        v2_last;
        v1_e_last;
        v2_e_last;
        %
        %         K_1 = 3;
        %         K_2 = 0.2;
        %         K_4 = 0.001;
        %         K_w = 0.7;
        %         T_w = 0.5;
        %K_r = 6e7;
        K_w_theta = pi;
        K_r = 3e5;
        Repulsive_Distance = 500;
        alpha_r = 0.3;
        K_v = 0.7;
        K_trans = 12.7;
        T_v = 0.3;
        forwardSpeed = 20;
        
        K_p = 0.5;
        Direction_Strength = 2;
        L = 75.5;
        
    end
    
    methods
        
        function obj = Controller(fix)
            obj.thetaFix = fix;
            obj.lastTime = now;
            obj.v1_last = 0;
            obj.v2_last = 0;
            obj.v1_e_last = 0;
            obj.v2_e_last = 0;
        end
        
        function [v1, v2, reach_target, obj] = update(obj, trans, rotation, target, targetTheta, obstacles)
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
            
            
            
            if((x_T - trans(1))^2 + (y_T - trans(2))^2  < 1e4)
                v1 = 0;
                v2 = 0;
                reach_target = true;
                obj.v1_last = v1;
                obj.v2_last = v2;
                
                obj.v1_e_last = 0;
                obj.v2_e_last = 0;
                
            else
                % Obstacles avoidance
                %                 Grad = [2*obj.K_p*(obj.x(end)-x_T), 2*obj.K_p*(obj.y(end)-y_T)];
                Grad = [2*obj.K_p*(obj.Direction_Strength*sin(targetTheta)^2*(obj.x(end)-x_T)-(obj.Direction_Strength-1)*sin(targetTheta)*cos(targetTheta)*(obj.y(end)-y_T)+cos(targetTheta)^2*(obj.x(end)-x_T))/obj.Direction_Strength,...
                    2*obj.K_p*(obj.Direction_Strength*cos(targetTheta)^2*(obj.y(end)-y_T)-(obj.Direction_Strength-1)*sin(targetTheta)*cos(targetTheta)*(obj.x(end)-x_T)+sin(targetTheta)^2*(obj.y(end)-y_T))/obj.Direction_Strength];
                for i = 1:size(obstacles,1)
                    x_R = obstacles(i,1);
                    y_R = obstacles(i,2);
                    %                    if sqrt((obj.x(end)-x_R)^2 + (obj.y(end)-y_R)^2) < obj.Repulsive_Distance
                    Grad = Grad + obj.K_r * ...
                        [...
                        (obj.Repulsive_Distance / ((obj.x(end)-x_R)^2 + (obj.y(end)-y_R)^2)^(1/2))^(obj.alpha_r) / ((obj.x(end)-x_R)^2 + (obj.y(end)-y_R)^2) * (x_R-obj.x(end)),...
                        (obj.Repulsive_Distance / ((obj.x(end)-x_R)^2 + (obj.y(end)-y_R)^2)^(1/2))^(obj.alpha_r) / ((obj.x(end)-x_R)^2 + (obj.y(end)-y_R)^2) * (x_R-obj.y(end))...
                        ];
                    %                    end
                end
                
                v_d = sqrt(Grad(1)^2 + Grad(2)^2);
                % v_d = obj.forwardSpeed * obj.K_trans;
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
                
                v = v_d; w = w_d;
                v1_d = (v - obj.L*w);
                v2_d = (v + obj.L*w);
                
                v1_e = v1_d - obj.v1_last;
                v2_e = v2_d - obj.v2_last;
                
                v1 = obj.v1_last + obj.K_v*(v1_e - obj.v1_e_last) + (obj.K_v*dt/obj.T_v)*v1_e;
                v2 = obj.v2_last + obj.K_v*(v2_e - obj.v2_e_last) + (obj.K_v*dt/obj.T_v)*v2_e;
                
                %                 v1 = v1_d;
                %                 v2 = v2_d;
                if((x_T - trans(1))^2 + (y_T - trans(2))^2  < 1e5)
                    reach_target = true;
                else
                    reach_target = false;
                end
                obj.v1_last = v1;
                obj.v2_last = v2;
                
                obj.v1_e_last = v1_e;
                obj.v2_e_last = v2_e;
            end
            
            
            
        end
        
    end
    
end
