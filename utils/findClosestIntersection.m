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

function [intersection_name, lat, long, px, py] = findClosestIntersection(GPS_data, intersections)
% FINDCLOSESTINTERSECTION given GPS coordinate of the user, it retrieves
% the closest intersection from the DB.
% INPUT: 
%   GPS_DATA: a vector containing latitude and longitude of the position of 
%             the user
%   INTERSECTIONS: the JSON file containing the intersections
% OUTPUT: 
%   INTERSECTION_NAME - string identifying the retrieved intersection
%   LAT, LONG - latitude and longitude of the retrieved intersection
%   PX, PY - image coordinate of the point in the template corresponding to
%            LAT and LONG

intersection_name = '';
px = 0;
py = 0;
lat = 0;
long = 0;
if ~isempty(intersections)
    
    minDist = 1000;
    minIdx = -1;
    for i = 1 : length(intersections.intersections)
        lat = intersections.intersections{i}.lat;
        long = intersections.intersections{i}.long;
        d = sqrt((lat - GPS_data(1))^2 + (long - GPS_data(2))^2);
        if d < minDist
            minDist = d;
            minIdx = i;
        end
    end
    
    if minIdx > 0
        intersection_name = intersections.intersections{minIdx}.name;
        lat = intersections.intersections{minIdx}.lat;
        long = intersections.intersections{minIdx}.long;
        px = intersections.intersections{minIdx}.x;
        py = intersections.intersections{minIdx}.y;
    end
    
end

end