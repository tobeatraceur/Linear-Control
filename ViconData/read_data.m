function [output] = read_data(MyClient)

    output = []

    % Get a frame
    fprintf( 'Waiting for new frame...' );
    while MyClient.GetFrame().Result.Value ~= Result.Success
        fprintf( '.' );
    end% while
    fprintf( '\n' );  

    % Count the number of subjects
    SubjectCount = MyClient.GetSubjectCount().SubjectCount;

    for SubjectIndex = 1:SubjectCount

        % Get the subject name
        SubjectName = MyClient.GetSubjectName( SubjectIndex ).SubjectName;

        % Count the number of segments
        SegmentCount = MyClient.GetSegmentCount( SubjectName ).SegmentCount;

        for SegmentIndex = 1:SegmentCount

            segmentData = containers.Map;
            segmentData('SubjectName') = SubjectName;

            % Get the segment name
            SegmentName = MyClient.GetSegmentName( SubjectName, SegmentIndex ).SegmentName;
            segmentData('SegmentName') = SegmentName;

            % Get the global segment translation
            Output_GetSegmentGlobalTranslation = MyClient.GetSegmentGlobalTranslation( SubjectName, SegmentName );
            segmentData('GlobalTranslation') = Output_GetSegmentGlobalTranslation;

            % Get the global segment rotation in quaternion co-ordinates
            Output_GetSegmentGlobalRotationQuaternion = MyClient.GetSegmentGlobalRotationQuaternion( SubjectName, SegmentName );
            segmentData('GlobalQuaternion') = Output_GetSegmentGlobalRotationQuaternion;

            % Get the global segment rotation in EulerXYZ co-ordinates
            Output_GetSegmentGlobalRotationEulerXYZ = MyClient.GetSegmentGlobalRotationEulerXYZ( SubjectName, SegmentName );
            segmentData('GlobalEuler') = Output_GetSegmentGlobalRotationEulerXYZ;

            output = [output, segmentData];

        end% SegmentIndex

    end% SubjectIndex

end
