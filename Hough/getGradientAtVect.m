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

function [g, a, y, x, idx] = getGradientAtVect(G, Gdir, col, row, angle, radius)
%returns the interpolated G value of the point that has distance radius from
%the point x,y along the direction specified by angle.



g = zeros(length(col),1);
a= angle;
xoff = radius*cosd(angle);   % x and y offset of points at specified radius and angle
yoff = radius*sind(angle);   % from each reference position.

hfrac = xoff - floor(xoff); % Fractional offset of xoff relative to integer location
vfrac = yoff - floor(yoff); % Fractional offset of yoff relative to integer location

x = col + xoff;     % x, y location on one side of the point in question
y = row - yoff;

idx = find(x>2 & x<size(G,2)-2 & y > 2 & y < size(G,1)-2);

fx = floor(x(idx));          % Get integer pixel locations that surround location x,y
cx = ceil(x(idx));
fy = floor(y(idx));
cy = ceil(y(idx));
tl = G(sub2ind(size(G),fy,fx));    % Value at top left integer pixel location.
tr = G(sub2ind(size(G),fy,cx));    % top right
bl = G(sub2ind(size(G),cy,fx));    % bottom left
br = G(sub2ind(size(G),cy,cx));    % bottom right

upperavg = tl + hfrac(idx) .* (tr - tl);  % Now use bilinear interpolation to
loweravg = bl + hfrac(idx) .* (br - bl);  % estimate value at x,y
g(idx) = upperavg + vfrac(idx) .* (loweravg - upperavg);

tl = Gdir(sub2ind(size(Gdir),fy,fx));    % Value at top left integer pixel location.
tr = Gdir(sub2ind(size(Gdir),fy,cx));    % top right
bl = Gdir(sub2ind(size(Gdir),cy,fx));    % bottom left
br = Gdir(sub2ind(size(Gdir),cy,cx));    % bottom right

upperavg = tl + hfrac(idx) .* (tr - tl);  % Now use bilinear interpolation to
loweravg = bl + hfrac(idx) .* (br - bl);  % estimate value at x,y
a(idx) = upperavg + vfrac(idx) .* (loweravg - upperavg);


end