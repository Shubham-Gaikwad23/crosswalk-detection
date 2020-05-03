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