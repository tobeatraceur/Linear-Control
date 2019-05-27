classdef Car

     properties
         bluetooth;
         MAX_SPEED = 50;
     end

     methods
         function obj = Car(name, channel)
             % class constructor
             obj.bluetooth = Bluetooth(name, channel);
             fopen(obj.bluetooth);
         end

         function command = set_speed(obj, speed)
             if(any(abs(speed) > obj.MAX_SPEED))
                 disp('speed too high!!');
             end
             speed(speed > obj.MAX_SPEED) = obj.MAX_SPEED;
             speed(speed < -obj.MAX_SPEED) = -obj.MAX_SPEED;
             command = sprintf('{333%+03d%+03d}', round(speed));
             %disp(['command: ', command]);
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
