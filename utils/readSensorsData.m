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