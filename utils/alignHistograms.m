function [maxTheta, flines] = alignHistograms(T, Tedge, J, F,...
                                                    meterXpixel, lines)
% ALIGNHISTOGRAMS aligns the histograms of the thetas that belong to the 
% the peaks of the Hough transform of T and F.
% 
% INPUT
%       T: template intersection
%       TEDGE: canny edges of T
%       J: composite aerial image
%       F: segmentation mask of J
%       METERXPIXEL: aerial image resolution  
%       LINES: lines extracted on the composite image using 3D Hough Transf
%
% OUTPUT:
%       MAXTHETA: the bearing correction angle
%       FLINES: set of lines without outliers thetas

linesT = findLinesStandardHough(T, Tedge, 16, .5, 10, 0);
linesF = findLinesStandardHough(uint8(imresize(J, 1/(.118/meterXpixel))), ...
    logical(uint8(imresize(F, 1/(.118/meterXpixel)))), 22,0.5,7, 0);

thetasT = zeros(1,length(linesT));
thetasF = zeros(1,length(linesF));

for lt = 1 : length(linesT)
    thetasT(lt) = linesT(lt).theta;
    if thetasT(lt) < 0
        thetasT(lt) = thetasT(lt)+180;
    end
end
for lf = 1 : length(linesF)
    thetasF(lf) = linesF(lf).theta;
    if thetasF(lf) < 0
        thetasF(lf) = thetasF(lf)+180;
    end
end

thetasL = zeros(1,size(lines,1));
for ll = 1 : size(lines,1)
    thetasL(ll) = mod(180-lines(ll,3),180);
end

NT = histc((mod(round(thetasT),360)),0:359);
NF = histc((mod(round(thetasF),360)),0:359);
NL = histc((mod(round(thetasL),360)),0:359);

alpha = 2;
gaussFilter1 = gausswin(25, alpha);
gaussFilter1 = gaussFilter1 / sum(gaussFilter1); % Normalize.
gaussFilter2 = gausswin(30, 1);
gaussFilter2 = gaussFilter2 / sum(gaussFilter2); % Normalize.

NT = NT(1:181)./sum(NT);
NF = NF(1:181)./sum(NF);
NL = NL(1:181)./sum(NL);
tmpNT = NT;
NT = conv(NT, gaussFilter1, 'same');
NTF = conv(tmpNT, gaussFilter2, 'same');

hcorr = cconv(NT, (fliplr(NF)));

delta = 25;
[~, i] = max(hcorr(180-delta:180+delta));
toc

base = 180-delta;
maxTheta = (180-(base+i)-1);

display('* Filtering outliers')
tic
NL = circshift(NL,[0 -maxTheta]).*(NTF==0);
NL = circshift(NL,[0 maxTheta]);
NL = NL*0; % comment this line to filter outliers thetas
filtIdx = NL > 0;
linesFilter = zeros(1,size(lines,1));
for i = find(filtIdx>0)
    angle = i-1;
    f = find(thetasL == angle);
    linesFilter(f) = 1;
end

flines = lines;
flines(linesFilter==1,:) = [];

end