% Program options
TransmitMulticast = false;
EnableHapticFeedbackTest = false;
HapticOnList = {'ViconAP_001';'ViconAP_002'};
bReadCentroids = false;

% A dialog to stop the loop
MessageBox = msgbox( 'Stop DataStream Client', 'Vicon DataStream SDK' );

% Load the SDK
fprintf( 'Loading SDK...' );
Client.LoadViconDataStreamSDK();
fprintf( 'done\n' );

% Program options
HostName = 'localhost:801';

% Make a new client
MyClient = Client();

% Connect to a server
fprintf( 'Connecting to %s ...', HostName );
while ~MyClient.IsConnected().Connected
    MyClient.Connect( HostName );
    fprintf( '.' );
end
fprintf( '\n' );

% Enable some different data types
MyClient.EnableSegmentData();
fprintf( 'Segment Data Enabled: %s\n',          AdaptBool( MyClient.IsSegmentDataEnabled().Enabled ) );

% Set the streaming mode
MyClient.SetStreamMode( StreamMode.ClientPull );

% Set the global up axis
MyClient.SetAxisMapping( Direction.Forward, ...
    Direction.Left,    ...
    Direction.Up );    % Z-up

filename = 'spin+10-10';
outputFile = fopen(['output/', filename, '.txt.'], 'w');

Counter = 1;
% Loop until the message box is dismissed
while ishandle( MessageBox )
    drawnow;
    Counter = Counter + 1;

    % Get a frame
    fprintf( 'Waiting for new frame...' );
    while MyClient.GetFrame().Result.Value ~= Result.Success
        fprintf( '.' );
    end% while
    fprintf( '\n' );  

    % Get the frame number
    Output_GetFrameNumber = MyClient.GetFrameNumber();
    fprintf( 'Frame Number: %d\n', Output_GetFrameNumber.FrameNumber );

    % Print time
    fprintf( outputFile, '\ntime: %.6f\n', now*60*60*24);
    
    % Get the timecode
    Output_GetTimecode = MyClient.GetTimecode();
    fprintf( 'Timecode: %dh %dm %ds %df %dsf %s %d %d %d\n\n',    ...
        Output_GetTimecode.Hours,                  ...
        Output_GetTimecode.Minutes,                ...
        Output_GetTimecode.Seconds,                ...
        Output_GetTimecode.Frames,                 ...
        Output_GetTimecode.SubFrame,               ...
        AdaptBool( Output_GetTimecode.FieldFlag ), ...
        Output_GetTimecode.Standard.Value,         ...
        Output_GetTimecode.SubFramesPerFrame,      ...
        Output_GetTimecode.UserBits );

    % Get the latency
    fprintf( 'Latency: %gs\n', MyClient.GetLatencyTotal().Total );

    % Count the number of subjects
    SubjectCount = MyClient.GetSubjectCount().SubjectCount;
    fprintf( 'Subjects (%d):\n', SubjectCount );
    for SubjectIndex = 1:SubjectCount
        fprintf( '  Subject #%d\n', SubjectIndex - 1 );

        % Get the subject name
        SubjectName = MyClient.GetSubjectName( SubjectIndex ).SubjectName;
        fprintf( '    Name: %s\n', SubjectName );

        % Get the root segment
        RootSegment = MyClient.GetSubjectRootSegmentName( SubjectName ).SegmentName;
        fprintf( '    Root Segment: %s\n', RootSegment );

        % Count the number of segments
        SegmentCount = MyClient.GetSegmentCount( SubjectName ).SegmentCount;
        fprintf( '    Segments (%d):\n', SegmentCount );
        for SegmentIndex = 1:SegmentCount
            fprintf( '      Segment #%d\n', SegmentIndex - 1 );

            % Get the segment name
            SegmentName = MyClient.GetSegmentName( SubjectName, SegmentIndex ).SegmentName;
            fprintf( '        Name: %s\n', SegmentName );

            % Get the global segment translation
            Output_GetSegmentGlobalTranslation = MyClient.GetSegmentGlobalTranslation( SubjectName, SegmentName );
            fprintf( outputFile, 'Global Translation: (%g, %g, %g) %s\n',               ...
                Output_GetSegmentGlobalTranslation.Translation( 1 ), ...
                Output_GetSegmentGlobalTranslation.Translation( 2 ), ...
                Output_GetSegmentGlobalTranslation.Translation( 3 ), ...
                AdaptBool( Output_GetSegmentGlobalTranslation.Occluded ) );
% 
%             % Get the global segment rotation in helical co-ordinates
%             Output_GetSegmentGlobalRotationHelical = MyClient.GetSegmentGlobalRotationHelical( SubjectName, SegmentName );
%             fprintf( '        Global Rotation Helical: (%g, %g, %g) %s\n',           ...
%                 Output_GetSegmentGlobalRotationHelical.Rotation( 1 ), ...
%                 Output_GetSegmentGlobalRotationHelical.Rotation( 2 ), ...
%                 Output_GetSegmentGlobalRotationHelical.Rotation( 3 ), ...
%                 AdaptBool( Output_GetSegmentGlobalRotationHelical.Occluded ) );
% 
%             % Get the global segment rotation as a matrix
%             Output_GetSegmentGlobalRotationMatrix = MyClient.GetSegmentGlobalRotationMatrix( SubjectName, SegmentName );
%             fprintf( '        Global Rotation Matrix: (%g, %g, %g, %g, %g, %g, %g, %g, %g) %s\n', ...
%                 Output_GetSegmentGlobalRotationMatrix.Rotation( 1 ),               ...
%                 Output_GetSegmentGlobalRotationMatrix.Rotation( 2 ),               ...
%                 Output_GetSegmentGlobalRotationMatrix.Rotation( 3 ),               ...
%                 Output_GetSegmentGlobalRotationMatrix.Rotation( 4 ),               ...
%                 Output_GetSegmentGlobalRotationMatrix.Rotation( 5 ),               ...
%                 Output_GetSegmentGlobalRotationMatrix.Rotation( 6 ),               ...
%                 Output_GetSegmentGlobalRotationMatrix.Rotation( 7 ),               ...
%                 Output_GetSegmentGlobalRotationMatrix.Rotation( 8 ),               ...
%                 Output_GetSegmentGlobalRotationMatrix.Rotation( 9 ),               ...
%                 AdaptBool( Output_GetSegmentGlobalRotationMatrix.Occluded ) );

            % Get the global segment rotation in quaternion co-ordinates
            Output_GetSegmentGlobalRotationQuaternion = MyClient.GetSegmentGlobalRotationQuaternion( SubjectName, SegmentName );
            fprintf( outputFile,  'Global Rotation Quaternion: (%g, %g, %g, %g) %s\n',             ...
                Output_GetSegmentGlobalRotationQuaternion.Rotation( 1 ),       ...
                Output_GetSegmentGlobalRotationQuaternion.Rotation( 2 ),       ...
                Output_GetSegmentGlobalRotationQuaternion.Rotation( 3 ),       ...
                Output_GetSegmentGlobalRotationQuaternion.Rotation( 4 ),       ...
                AdaptBool( Output_GetSegmentGlobalRotationQuaternion.Occluded ) );

            % Get the global segment rotation in EulerXYZ co-ordinates
            Output_GetSegmentGlobalRotationEulerXYZ = MyClient.GetSegmentGlobalRotationEulerXYZ( SubjectName, SegmentName );
            fprintf(  outputFile, 'Global Rotation EulerXYZ: (%g, %g, %g) %s\n',                 ...
                Output_GetSegmentGlobalRotationEulerXYZ.Rotation( 1 ),       ...
                Output_GetSegmentGlobalRotationEulerXYZ.Rotation( 2 ),       ...
                Output_GetSegmentGlobalRotationEulerXYZ.Rotation( 3 ),       ...
                AdaptBool( Output_GetSegmentGlobalRotationEulerXYZ.Occluded ) );

        end% SegmentIndex

    end% SubjectIndex

end% while true  
fclose(outputFile);
% Disconnect and dispose
MyClient.Disconnect();

% Unload the SDK
fprintf( 'Unloading SDK...' );
Client.UnloadViconDataStreamSDK();
fprintf( 'done\n' );
