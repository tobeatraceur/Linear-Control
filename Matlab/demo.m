%% Initiation
% Include necessary files
addpath(genpath('./include/'));

% Start Vicon client
client = VData();

% Initiate a car
car0 = Car('Bt04-A', 1);

% Open file
filename = 'testfile';
outputFile = fopen(['../ViconData/', filename, '.txt'], 'w');

% Log experiment time
fprintf(outputFile, 'Log time: %s\n\n', ...
    datetime('now','TimeZone','local','Format','d-MM-y HH:mm:ss'));

%% Send speed and log
% Send speed command
speed = [-2, -50]; % Set speed here
car0.set_speed(speed);

% Read data
% TODO: Check if this works properly.
data = client.read_data() 

% Write timestamps to file
fprintf(outputFile, '\ntime: %.6f\n', now*60*60*24);
% TODO: Log data

%% Close
% Close client, Bluetooth and output file
client.close_client();
car0.close();
fclose(outputFile);
