classdef Car

     properties
         bluetooth;
         MAX_SPEED = 50;
     end

     methods
         function obj = Car(name, channel)
             % Class constructor
             obj.bluetooth = Bluetooth(name, channel);
             fopen(obj.bluetooth);
         end

         function command = set_speed(obj, speed)
             if(sum(speed.^2) > 2 * obj.MAX_SPEED^2)
                 disp('speed too high!!');
                 speed = speed * sqrt (2 * obj.MAX_SPEED^2 / sum(speed.^2))
             end
             command = sprintf('{333%+03d%+03d}', round(speed));
             disp(['command: ', command]);
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
