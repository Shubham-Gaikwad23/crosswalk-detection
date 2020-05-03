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

function [H, C, Cw] = HoughTransformWidthFast(I, E, Mask, ...
    resolution_theta, delta_theta, resolution_d, minradius, maxradius, ...
    radius_step, gradientRange, normGradient, nonMaxima, angleTol, ...
    interpolateEdges, Haar)

%HOUGHTRANSFORMWIDTH computes a modified Hough transform to detect stripes 
% in a specific range of widths.

% Input:    I - Input image
%           E - edge map (optional , [] if not available)
%           Mask - binary mask defining ROI (optional, [] if none
%                  available)
%           resolution_theta - resolution of the angle used in the Hough
%                              transform (degrees)
%           delta_theta - half range of thetas to examine given the
%                          orientation of an edge point (degrees)
%           resolution_d - resolution of the distance in Hough transform
%           minradius - minimum width of a stripe to be detected
%           maxradius - maximum width of a stripe to be detected
%           radius_step - increment step of the width
%           gradientRange - 2D vector specifying min and max gradient value
%                           for a point to be considered as a valid edge
%                           point
%           normGradient - if set to 1 the gradient values will be
%                          normalized between 0 and 1
%           nonMaxima - if set to 1 non maxima suppression will be executed
%                       on the gradient image
%           minPeak - minimum size of a peak in the Hough transform to be
%                     considered as a real line
%           angleTol - tolerance on the 'conjugate angles' constraint
%                      (degrees)
%           interpolateEdges - if 1 the interpolation is used when
%                              searching for a matching edge point along 
%                              the gradient direction
%           Haar -  Haar filter convolution map.

% Output:   H - Hough Transform H(rho, theta, width)
%           
%           C - for each peak in H, it contains the points who contributed
%           to the voting of that line. Is indexed using the bins indexes
%           Cw - corresponding points to the points in C


% addpath('../MatlabFns/Spatial');

% Important: Angle measures are in DEGREES
if size(I,3)> 1
    I = rgb2gray(I);
end

% [GmagInt, ~] = imgradient(I.*255,'CentralDifference');
I = im2double(I);
% [Gmag, Gdir] = imgradient(I,'IntermediateDifference');

h = [-1 1];
dx = imfilter(I,h);
dy = imfilter(I,h');

gaus = fspecial('gaussian', [3 3], .5);
dx = imfilter(dx,gaus,'same','replicate');
dy = imfilter(dy,gaus,'same','replicate');

Gmag = sqrt(dx.^2 + dy.^2);
Gdir = atan2d(-dy,dx);



if isempty(E)
    E = edge(I,'canny');
end
% 
E(isnan(E)) = 0;
E = logical(E); 

if nonMaxima %non maxima suppression of the gradient (reduce thick gradient edges)
    Gdd = Gdir+180;
    Gdd(Gdd>180) = Gdd(Gdd>180) - 180;
    [Gmag, ~] = nonmaxsup(Gmag, Gdd, 1.2);
end

Gdir = mod(Gdir,360); % make gradient direction go between [0-360)

if normGradient %normalize gradient between [0-1]
    Gmag = (Gmag - min(min(Gmag))) ./ (max(max(Gmag)) - min(min(Gmag)));
end

if ~isempty(Mask)
    Gbin = zeros(size(Gmag));
    Gbin(Gmag>=gradientRange(1) & Gmag<=gradientRange(2)) = 1;
    Gbin = bwmorph(Gbin, 'clean', inf);
    Gmag = Gmag .* double(Gbin) .* double(Mask);
end

%max distance for lines in polar coordinates (2*image diagonal)
maxd = floor(sqrt(size(I,1)^2 + size(I,2)^2));

H = zeros(floor(2*maxd/resolution_d), floor(360/resolution_theta),ceil((maxradius-minradius+1)/radius_step));

C = cell(size(H));
Cw = cell(size(H));

Gmag(1:3,:) = 0;
Gmag(end-3:end,:) = 0;

idx1 = Gmag>=gradientRange(1);
idx2 = Gmag<=gradientRange(2);
idx3 = E>0;
idx = find(idx1.*idx2.*idx3>0);

theta0 = Gdir(idx);
thetas = double(getThetaRangeVec(theta0, delta_theta, resolution_theta));

[row, col] = ind2sub(size(E),idx);
x = col - 1;
y = size(I,1)-row;
d = cosd(thetas).*repmat(x,1,size(thetas,2)) + sind(thetas).*repmat(y,1,size(thetas,2));
theta_idx = floor(thetas/resolution_theta); %it was +1
theta_idx(theta_idx == 0) = 360; %new line
d_idx = d2index(d, resolution_d, maxd);


for dx = 1 : size(d_idx,2)
    rcnt = 0;
%     if (d_idx(dx)>0) && (d_idx(dx)<size(H,1))
        for radius = minradius : radius_step : maxradius
            [g, angle, r, c, ~] = getGradientAtVect(Gmag, Gdir, col, row, thetas(:,dx), radius);
            conj = (cosd(thetas(:,dx) - angle-180) > cosd(angleTol));
            rcnt = rcnt +1;
            [e, ~] = interpolateValueAtVect(c,r,E, interpolateEdges);
            edgePoint = e >0;
            gg = g>= gradientRange(1) & g <= gradientRange(2);
            voterIdx = find(conj.*edgePoint.*gg>0);
            voterIdx = voterIdx(d_idx(voterIdx,dx)>0 & d_idx(voterIdx,dx)<size(H,1));
           
            for v = voterIdx'
                rmean = floor( ((size(I,1)-y(v)) + (r(v)))/2);
                cmean = floor((x(v)+1 + c(v)) / 2);
                if ~isempty(Haar)
                    hval = Haar( rmean, cmean);
                else
                    hval = 6e3;
                end
                if hval > 5e3
                     H(d_idx(v,dx),theta_idx(v,dx),rcnt) = H(d_idx(v,dx),theta_idx(v,dx),rcnt) + 1;
                    if ~isempty(C{d_idx(v,dx),theta_idx(v,dx),rcnt})
                        C{d_idx(v,dx),theta_idx(v,dx),rcnt} = [ C{d_idx(v,dx),theta_idx(v,dx),rcnt}; y(v) x(v)];
                        Cw{d_idx(v,dx),theta_idx(v,dx),rcnt} = [ Cw{d_idx(v,dx),theta_idx(v,dx),rcnt}; size(I,1)-r(v) c(v)];
                    else
                        C{d_idx(v,dx),theta_idx(v,dx),rcnt} = [y(v) x(v)];
                        Cw{d_idx(v,dx),theta_idx(v,dx),rcnt} = [size(I,1)-r(v) c(v)];
                    end
                end
            end
            
        end
%     end
end

end
