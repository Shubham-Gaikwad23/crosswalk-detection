function [lines, Htotal] = extractPeaksLinesGlobal(H,minPeak, resolution_theta,...
    resolution_d, maxd)
% EXTRACTPEAKSLINES: Extract lines associated to Hough peaks
% INPUT:
%           H - Hough transform accumulator
%           MINPEAK - minimum value for a peak to be considered a line
%                     candidate
%           RESOLUTION_THETA - theta sampling resolution used in the Hough 
%                              transform
%           RESOLUTION_D - d sampling resolution
%           MAXD - max value for D in the Hough transform
% OUTPUT:
%           LINES - retrieved lines in the form lines = ( : , [peak value,
%                   rho, theta])
%           MAXD - max value for the 'd' parameter in the Hough   

% The function houghpeaks_Floats is a modified version of the matlab
% function to detect peaks in the hough that accepts non integer hough
% accumuator as input.

lines = [];
[Htotal, W] = max(H,[],3);

W(Htotal==0)=0;
if minPeak > 0
    peaks = houghpeaks_Floats(Htotal(:,:), 80,'Threshold',minPeak,'NHoodSize', [9 9]);
else
    peaks = houghpeaks_Floats(Htotal(:,:),80, 'NHoodSize', [8 8]);
end

for l = 1 : size(peaks,1)
    
    d_idx = peaks(l,1);
    t_idx = peaks(l,2);
    
    theta = (t_idx-1)*resolution_theta;
    d = index2d(d_idx, resolution_d, maxd);
    
    lines(end+1,:) = [Htotal(d_idx,t_idx) d theta -W(d_idx,t_idx)];
end
end

