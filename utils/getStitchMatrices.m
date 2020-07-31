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