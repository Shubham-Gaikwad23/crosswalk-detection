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

function lines = extractPeaksLinesGlobalSum(H,minPeak, resolution_theta,...
    resolution_d, maxd, numRadius)
% EXTRACTPEAKSLINES: Extract lines associated to Hough peaks
% OUTPUT:
%   lines - retrieved lines in the form lines = ( : , [peak value,
%   rho, theta])

lines = [];

Htotal = sum(sum(H,3),3);
W(Htotal==0)=0;
    if minPeak > 0
        peaks = houghpeaks_Floats(Htotal(:,:), 1000,'Threshold',minPeak,'NHoodSize', [3 3]);
    else
        peaks = houghpeaks_Floats(Htotal(:,:),80, 'NHoodSize', [8 8]);
    end

    for l = 1 : size(peaks,1)

        d_idx = peaks(l,1);
        t_idx = peaks(l,2);

        theta = (t_idx-1)*resolution_theta;
        d = index2d(d_idx, resolution_d, maxd);

        lines(end+1,:) = [Htotal(d_idx,t_idx) d theta];
    end
end

