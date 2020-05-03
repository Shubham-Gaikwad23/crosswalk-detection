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

function [status, idx, R] = getStitchMatrices(folder,img_list, mask_list, isPortrait)
% GETSTITCHMATRICES performes the stitching of the images in IMG_LIST in
% the folder FOLDER. MASK_LIST defines the ROI to use for the stitching for
% each image. Each ROI is described by a string using the following format:
% row_col_height_width. If MASK_LIST is empty, the ROI are set to the whole
% image; isPortrait specifies whether the images are shot in landscape (0) or
% potrait mode (1)
% The output is a variable STATUS that returns the exit code for the opencv
% stitching, IDX is a vector containing the indeces of the images actually
%used in during the stitching, R is a vector of the rotation matrices
%computed by the stitching.



if ~isempty(mask_list)
    if ~isPortrait
        command = strcat('.\stitcher\ImageStitching.exe ', ...
            img_list, ' ', mask_list, ' --usemasks --warp cylindrical --preview --blend no --expos_comp no --seam no --match_conf .5');
    else
        command = strcat('.\stitcher\ImageStitching.exe ', ...
            img_list, ' ', mask_list, ' --usemasks --warp cylindrical --preview --blend no --expos_comp no --seam no --match_conf .5 --rotate -90');
    end
else
    if ~isPortrait
        command = strcat('.\stitcher\ImageStitching.exe ', ...
            img_list, ' --warp cylindrical --preview --blend no --expos_comp no --seam no --match_conf .5');
    else
        command = strcat('.\stitcher\ImageStitching.exe ', ...
            img_list, ' --warp cylindrical --preview --blend no --expos_comp no --seam no --match_conf .5 --rotate -90');
    end
end


[status,cmdout] = dos(command);
stitchOut = cmdout;
R = [];
save(strcat(folder,'/','stitchingOutput.mat'),'stitchOut');
C = strsplit(stitchOut,{'[',']', ';',',', '\n'});
ind=find(ismember(C,' '));
idxlim = ind(1)-10;
idx = str2mat(C{1:idxlim});
idx = str2num(idx)+1;
C = C(idxlim+1:end);
CC = C(~cellfun('isempty',C));
rcnt = 0;
for r = 1 : 10 : length(CC)
    rcnt = rcnt + 1;
    R{rcnt} = reshape(str2double(CC(r:r+8)),[3 3]);
end
movefile('result.jpg', fullfile(folder,'panorama.jpeg'));
save(strcat(folder,'/','R_stitching.mat'),'R');
save(strcat(folder,'/','Indices_stitching.mat'),'idx');
end