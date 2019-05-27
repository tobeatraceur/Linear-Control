classdef VData
    properties
        MyClient;
        data;
    end

    methods
        function obj = VData()
            % Class constructor

            % Program options
%             TransmitMulticast = false;
%             EnableHapticFeedbackTest = false;
%             HapticOnList = {'ViconAP_001';'ViconAP_002'};
%             bReadCentroids = false;

            obj.data = containers.Map;

            % Load the SDK
            fprintf( 'Loading SDK...' );
            Client.LoadViconDataStreamSDK();
            fprintf( 'done\n' );

            % Program options
            HostName = 'localhost:801';

            % Make a new client
            obj.MyClient = Client();

            % Connect to a server
            while ~obj.MyClient.IsConnected().Connected
                obj.MyClient.Connect( HostName );
                % fprintf( '.' );
            end
            fprintf( '\n' );

            % Enable some different data types
            obj.MyClient.EnableSegmentData();

            % Set the streaming mode
            obj.MyClient.SetStreamMode( StreamMode.ClientPull );

            % Set the global up axis
            obj.MyClient.SetAxisMapping( Direction.Forward, ...
                Direction.Left,    ...
                Direction.Up );    % Z-up
        end

        function obj = read_data(obj)
            % Output format: list of map, each map has these key:
            %'SubjectName', 'SegmentName', 'GlobalTranslation', 'GlobalQuaternion', 'GlobalEuler'

            obj.data = containers.Map;

            % Get a frame
            % fprintf( 'Waiting for new frame...' );
            while obj.MyClient.GetFrame().Result.Value ~= Result.Success
                % fprintf( '.' );
            end
            % fprintf( '\n' );  

            % Count the number of subjects
            SubjectCount = obj.MyClient.GetSubjectCount().SubjectCount;

            for SubjectIndex = 1:SubjectCount

                % Get the subject name
                SubjectName = obj.MyClient.GetSubjectName( SubjectIndex ).SubjectName;

                % Count the number of segments
                SegmentCount = obj.MyClient.GetSegmentCount( SubjectName ).SegmentCount;

                for SegmentIndex = 1:SegmentCount

                    segmentData = containers.Map;
                    segmentData('SubjectName') = SubjectName;

                    % Get the segment name
                    SegmentName = obj.MyClient.GetSegmentName( SubjectName, SegmentIndex ).SegmentName;
                    segmentData('SegmentName') = SegmentName;

                    % Get the global segment translation
                    Output_GetSegmentGlobalTranslation = ...
                        obj.MyClient.GetSegmentGlobalTranslation( SubjectName, SegmentName );
                    segmentData('GlobalTranslation') = Output_GetSegmentGlobalTranslation;

                    % Get the global segment rotation in quaternion co-ordinates
                    Output_GetSegmentGlobalRotationQuaternion = ...
                        obj.MyClient.GetSegmentGlobalRotationQuaternion( SubjectName, SegmentName );
                    segmentData('GlobalQuaternion') = Output_GetSegmentGlobalRotationQuaternion;

                    % Get the global segment rotation in EulerXYZ co-ordinates
                    Output_GetSegmentGlobalRotationEulerXYZ = ...
                        obj.MyClient.GetSegmentGlobalRotationEulerXYZ( SubjectName, SegmentName );
                    segmentData('GlobalEuler') = Output_GetSegmentGlobalRotationEulerXYZ;

                    obj.data(segmentData('SegmentName')) = segmentData;

                end% SegmentIndex

            end% SubjectIndex
        end

        function translation = get_translation(obj, name)
            if isKey(obj.data, name)
                segment = obj.data(name);
                translation = segment('GlobalTranslation').Translation;
            else
                error('Object %s does not exist!', name)
            end
        end

        function obstacles = get_obstacles(obj, name)
            if isKey(obj.data, name)
                obstacles = [];
                for k = obj.data.keys()
                    if ~ strcmp(k{1}, name)
                        obstacle = obj.data(k{1});
                        obstacleTranslation = obstacle('GlobalTranslation').Translation;
                        obstacles = [obstacles; obstacleTranslation'];
                    end
                end
            else
                error('Object %s does not exist!', name)
            end
        end

        function [angle, rotation] = get_rotation(obj, name)
            if isKey(obj.data, name)
                segment = obj.data(name);
                rotation = segment('GlobalEuler').Rotation;
                angle = rotation(3);
            else
                error('Object %s does not exist!', name)
            end
        end

        function close_client(obj)
            % Disconnect and dispose
            obj.MyClient.Disconnect();

            % Unload the SDK
            fprintf( 'Unloading SDK...' );
            Client.UnloadViconDataStreamSDK();
            fprintf( 'done\n' );
        end
    end

end
