% Copyright (c) 2015, The Smith-Kettlewell Eye Research Institute
% All rights reserved.
% 
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
%     * Redistributions of source code must retain the above copyright
%       notice, this list of conditions and the following disclaimer.
%     * Redistributions in binary form must reproduce the above copyright
%       notice, this list of conditions and the following disclaimer in the
%       documentation and/or other materials provided with the distribution.
%     * Neither the name of the The Smith-Kettlewell Eye Research Institute nor
%       the names of its contributors may be used to endorse or promote products
%       derived from this software without specific prior written permission.
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
% ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
% WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
% DISCLAIMED. IN NO EVENT SHALL THE SMITH-KETTLEWELL EYE RESEARCH INSTITUTE BE
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
% DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
% LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
% ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
% (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
% SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

function [upVector, compass, quaternion, magnetic, R, GPS] = readSensorsData(folder, filename)
%READSENSORDATA reads the sensors data from the Android application.
%
% INPUT
%   folder - sensor data folder
%   filename - sensor data filename
%   ver (1|2) - specify which version of sensor file to read

[~, name, ~] = fileparts(filename);
accFile = strcat(folder,'/',name,'.txt');
fileId = fopen(accFile,'r');
GPS = [];

%autodetect version (to handle old sensor data file format)
upVector = textscan(fileId, 'Accl: \n %f %f %f %f \n \n' );
if isnan(upVector{2})
    ver = 1;
else ver = 2;
end

fclose(fileId);
fileId = fopen(accFile,'r');

if ver==2
    upVector = textscan(fileId, 'Accl: \n %f %f %f %f \n \n' );
    if isempty(upVector{2})
        ver = 3;
    end
end

fclose(fileId);
fileId = fopen(accFile,'r');

if ver == 1
     upVector = textscan(fileId, 'Accl: \n %f \n %f \n %f \n %f' );
    compass = textscan(fileId, 'Compass: \n %f \n %f \n %f ' );
    magnetic = textscan(fileId, 'Magnetic: \n %f \n %f \n %f \n %f' );
    quaternion = [];
    R = [];
    
    magnetic = cell2mat(magnetic);
    upVector = cell2mat(upVector);
    compass = cell2mat(compass);
    
elseif ver == 2
    upVector = textscan(fileId, 'Accl: \n %f %f %f %f \n \n' );
    compass = textscan(fileId, 'Compass: \n %f %f %f \n \n' );
    quaternion = textscan(fileId, 'Rotation: \n %f %f %f %f \n \n' );
    magnetic = textscan(fileId, 'Magnetic: \n %f %f %f %f \n \n' );
    
    quaternion = cell2mat(quaternion);
    upVector = cell2mat(upVector);
    magnetic = cell2mat(magnetic);
    compass = cell2mat(compass);
    
    R = ...
        textscan(fileId, ...
        ['Rotation Matrix: \n %f %f %f %f \n %f %f %f %f \n ', ...
        '%f %f %f %f \n %f %f %f %f'] );
    R = cell2mat(R);
    
    R = reshape(R,4,4);
    R = R';

elseif ver == 3
    upVector = textscan(fileId, 'Accl: %f %f %f %f \n \n' );
    compass = textscan(fileId, 'Compass: %f %f %f \n' );
    quaternion = textscan(fileId, 'Rotation: %f %f %f \n' );
    magnetic = textscan(fileId, 'Magnetic: %f %f %f %f \n \n' );
    
    quaternion = cell2mat(quaternion);
    upVector = cell2mat(upVector);
    magnetic = cell2mat(magnetic);
    compass = cell2mat(compass);
    
    R = ...
        textscan(fileId, ...
        ['Rotation Matrix: \n %f %f %f \n %f %f %f \n ', ...
        '%f %f %f \n'] );
    R = cell2mat(R);
    
    R = reshape(R,3,3);
    R = R';
    
    fclose(fileId);
    fileId = fopen(accFile,'r');
    stuff = textscan(fileId, '%s %s %f64 %f64' );

    longitude = stuff{3}(11);
    latitude = stuff{3}(12);
    altitude = stuff{3}(13);
    accuracy = stuff{3}(14);

    GPS = cell2mat(GPS);
    GPS(1) = latitude;
    GPS(2) = longitude;
    GPS(3) = altitude;
    GPS(4) = accuracy;
    
end

fclose(fileId);
end