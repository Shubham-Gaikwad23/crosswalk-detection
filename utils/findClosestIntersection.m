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