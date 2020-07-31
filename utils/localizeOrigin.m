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

