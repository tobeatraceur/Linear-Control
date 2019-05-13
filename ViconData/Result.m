% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Copyright (C) OMG Plc 2009.
% All rights reserved.  This software is protected by copyright
% law and international treaties.  No part of this software / document
% may be reproduced or distributed in any form or by any means,
% whether transiently or incidentally to some other use of this software,
% without the written permission of the copyright owner.
%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Part of the ViconDataStream SDK for MATLAB.
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
classdef Result
  properties (Constant = true)
    Unknown                            = 0;
    NotImplemented                     = 1;
    Success                            = 2;
    InvalidHostName                    = 3;
    InvalidMulticastIP                 = 4;
    ClientAlreadyConnected             = 6;
    ClientConnectionFailed             = 7;
    ServerAlreadyTransmittingMulticast = 8;
    ServerNotTransmittingMulticast     = 9;
    NotConnected                       = 10;
    NoFrame                            = 11;
    InvalidIndex                       = 12;
    InvalidCameraName                  = 13;
    InvalidSubjectName                 = 14;
    InvalidSegmentName                 = 15;
    InvalidMarkerName                  = 16;
    InvalidDeviceName                  = 17;
    InvalidDeviceOutputName            = 18;
    InvalidLatencySampleName           = 19;
    CoLinearAxes                       = 20;
    LeftHandedAxes                     = 21;
    HapticAlreadySet                   = 22;
  end
  
  properties
    Value
  end
  
  methods
    function obj = Result( value )
      obj.Value = value;
    end% Constructor
  end% methods
  
end% classdef
