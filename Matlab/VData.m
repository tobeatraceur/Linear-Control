classdef VData < handle
    properties
        MyClient;
        data;
        fig;
        ax;
        history;
        MAXPOINTS = 300;
        mousePos;
        readMouse = false;
    end

    methods
        function obj = VData()
            % Class constructor

            % Initialize figure
            obj.fig = figure();
            obj.ax = axes('Parent', obj.fig);

            % Initialize history
            obj.history = containers.Map;

            % Program options
            TransmitMulticast = false;
            EnableHapticFeedbackTest = false;
            HapticOnList = {'ViconAP_001';'ViconAP_002'};
            bReadCentroids = false;

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
            end

            % Enable some different data types
            obj.MyClient.EnableSegmentData();

            % Set the streaming mode
            obj.MyClient.SetStreamMode( StreamMode.ClientPull );

            % Set the global up axis
            obj.MyClient.SetAxisMapping( Direction.Forward, ...
                Direction.Left,    ...
                Direction.Up );    % Z-up
        end

        function ready = read_data(obj)
            % Output format: list of map, each map has these key:
            %'SubjectName', 'SegmentName', 'GlobalTranslation', 'GlobalQuaternion', 'GlobalEuler'

            readData= containers.Map;

            for retry = 1:5

                ready = true;

                % Get a frame
                while obj.MyClient.GetFrame().Result.Value ~= Result.Success
                end

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

                        if segmentData('GlobalTranslation').Occluded ~= 0
                            ready = false;
                        end

                        % Get the global segment rotation in quaternion co-ordinates
                        Output_GetSegmentGlobalRotationQuaternion = ...
                            obj.MyClient.GetSegmentGlobalRotationQuaternion( SubjectName, SegmentName );
                        segmentData('GlobalQuaternion') = Output_GetSegmentGlobalRotationQuaternion;

                        % Get the global segment rotation in EulerXYZ co-ordinates
                        Output_GetSegmentGlobalRotationEulerXYZ = ...
                            obj.MyClient.GetSegmentGlobalRotationEulerXYZ( SubjectName, SegmentName );
                        segmentData('GlobalEuler') = Output_GetSegmentGlobalRotationEulerXYZ;

                        readData(segmentData('SegmentName')) = segmentData;

                    end% SegmentIndex

                end% SubjectIndex

                % Update data only when all subjects are included
                if ready
                    obj.data = readData;
                    break;
                end

            end% All subject read

            if ready
                % Update history
                obj.update_history();
            else
                warning('Fail to find all subjects! Please check vicon system.')
            end
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
            if strcmp(name, 'none') == 1
                obstacles = [];
            elseif isKey(obj.data, name)
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

        function update_history(obj)
            for k = obj.data.keys()
                object = obj.data(k{1});
                objectTranlation = object('GlobalTranslation').Translation;

                if object('GlobalTranslation').Occluded == 0
                    if ~isKey(obj.history, k{1})
                        obj.history(k{1}) = [];
                    end

                    obj.history(k{1}) = [obj.history(k{1}); objectTranlation'];

                end
            end
        end

        function update_trajectory(obj)

            cla(obj.ax);
            hold on;
            
            for k = obj.data.keys()
                if ~isKey(obj.history, k{1})
                    continue;
                end
                his= obj.history(k{1});
                x = his(1:end, 1);
                y = his(1:end, 2);

                % Draw only recent trajectory
                if length(x) > obj.MAXPOINTS
                    x = x(end-obj.MAXPOINTS+1 : end);
                    y = y(end-obj.MAXPOINTS+1 : end);
                end

                if obj.readMouse
                    plot(obj.ax, obj.mousePos(1), obj.mousePos(2), 'go')
                end
                
                plot(obj.ax, x, y);
                plot(obj.ax, x(end), y(end), '+');
%                 rectangle([-1700 -2000 2000 2850]);
                axis([-2200 1000 -2200 1500]);
            drawnow;
            end
            hold off;

        end

        function cp = get_mouse(obj)
            cp = get(obj.ax, 'CurrentPoint');
            cp = [cp(1, 1:2), 0];
            obj.mousePos = cp;
            if (cp(1) < 300) && (cp(1) > -1700) && (cp(2) < 850) && (cp(2) > -2000)
                obj.readMouse = true;
            else
                obj.readMouse = false;
            end
        end
    end

end
