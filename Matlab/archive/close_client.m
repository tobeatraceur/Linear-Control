function [] = close_client(MyClient)
    % Disconnect and dispose
    MyClient.Disconnect();

    % Unload the SDK
    fprintf( 'Unloading SDK...' );
    Client.UnloadViconDataStreamSDK();
    fprintf( 'done\n' );
end
