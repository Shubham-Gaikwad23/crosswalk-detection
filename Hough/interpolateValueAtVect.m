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

function [val, idx] = interpolateValueAtVect(x,y,I, interpolate)
val = zeros(length(x),1);
idx = find(x >=1 & x <= size(I,2) & y >= 1 & y <= size(I,1));

if interpolate
    hfrac = x(idx) - floor(x(idx)); % Fractional offset of xoff relative to integer location
    vfrac = y(idx) - floor(y(idx)); % Fractional offset of yoff relative to integer location
    
    
    fx = floor(x(idx));          % Get integer pixel locations that surround location x,y
    cx = ceil(x(idx));
    fy = floor(y(idx));
    cy = ceil(y(idx));
    tl = I(sub2ind(size(I),fy,fx));    % Value at top left integer pixel location.
    tr = I(sub2ind(size(I),fy,cx));    % top right
    bl = I(sub2ind(size(I),cy,fx));    % bottom left
    br = I(sub2ind(size(I),cy,cx));    % bottom right
    
    upperavg = tl + hfrac .* (tr - tl);  % Now use bilinear interpolation to
    loweravg = bl + hfrac .* (br - bl);  % estimate value at x,y
    val(idx) = upperavg + vfrac .* (loweravg - upperavg);
else
    val(idx) = I(sub2ind(size(I),round(y(idx)),round(x(idx))));
end

end
