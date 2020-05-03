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

function [C, maxCorrValue, Sbest, Jrot, maxCorrAngle] = localizeOrigin(So,T, J, mxpix, offsets)
% LOCALIZEIMAGE
% Given an segmented aerial view and an intersection template, the function
% finds the rotation that maximizes the correlation between the
% segmentation and the template.
% INPUT
%       So: segmentation mask
%       T: edge map of the intersection template
%       J: aerial composite
%       MXPIX: meters per pixel (aerial view resolution)
%       OFFSETS: vector of rotations to consider for the correlation
%
% OUTPUT:
%   C: correlation map
%   MAXCORRVALUE: max correlation value
%   SBEST: segmentation mask rotated by the angle corresponding to the max
%          correlation value
%   JROT: rotated composite image
%   MAXCORRANGLE: angle that maximizes the correlation

if isempty(offsets)
    offsets = -15 : 1 : 15;
end
offidx = 0;
maxCorrValue = 0;
maxCorrIdx = 0;
maxCorrAngle = 0;

Sorig = So;
So = logical(uint8(imresize(So, 1/(.118/mxpix))));
Corr = cell(length(offsets));
CorrMat = zeros(size(T,1),size(T,2),length(offsets));
%% correlation with different rotation of the segmentation mask
for off = offsets
    offidx = offidx +1;
    S = imrotate(So,(off),'crop');
    Corr{offidx} = imfilter(double(T), double(S), 'same', 'replicate', 'corr');        
    CorrMat(:,:,offidx) = Corr{offidx};
    maxC = max(max(Corr{offidx}));

    if maxC >= maxCorrValue
        maxCorrValue = maxC;
        maxCorrIdx = offidx;
        Sbest = S;
    end
end
maxCorrAngle = offsets(maxCorrIdx);
C = Corr{maxCorrIdx};
if ~isempty(J)
    Jrot = imrotate(J,(offsets(maxCorrIdx)),'crop');
    Sbest = imrotate(Sorig,(offsets(maxCorrIdx)),'crop');    
end

end

