function [E, Cmax] = haarFiltering(Comp, Mask, thr)
% HAARFILTERING convolves the image COMP with a set of Haar filters. The
% convolution result is obtained considering the max convlution value for
% each pixel.
% INPUT:
% COMP image to segment, MASK is a binary mask that defines the ROI for
% COMP, THR is a binarization threshold for the convolution map.
% OUTPUT:
% E binary segmentation of the convolution map, CMAX is the convolution
% map.

C = [];
tcont = 0;
taurange = [22.3 82+14.3];
for w = 12 : 2 : 16
    M = cosineFilter(w, 5);

    for tau = taurange 
        tcont = tcont+1;
        Mrot = imrotate(M,tau,'loose','bilinear');
        C(:,:,tcont) = imfilter(Comp,Mrot,'same','conv','replicate');
        
    end
end

Cmax = max(C,[],3);
Cmax = Cmax.*logical(Mask);
E = zeros(size(Comp));
E(Cmax>thr) =  1;

end