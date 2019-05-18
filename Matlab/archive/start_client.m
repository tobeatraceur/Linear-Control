function [MyClient] = start_client():
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

end
