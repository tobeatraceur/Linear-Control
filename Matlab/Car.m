 classdef Car

     properties
         bluetooth;
     end

     methods
         function obj = Car(name, channel)
             % class constructor
             bluetooth = Bluetooth(name, channel);
             fopen(bluetooth);
         end

         function obj = set_speed(obj, speed)
             command = sprintf('{333%+03d%+03d}', speed)
             fprintf(obj.bluetooth, command);
         end

         function close(obj)
             obj.set_speed([0, 0]);
             fclose(obj.bluetooth);
         end
     end

 end
