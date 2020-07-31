function maskRectString = extractImageROI(tmpI, upVectorList, focal)
% extractImageROI returns the string describing the ROI below the horizon.
% maskRectString has the following format: row_col_height_width.

H = horizonMask(tmpI, upVectorList, focal);
[mr1, ~] = (find(H(:,1)>0));
[mr2, ~] = (find(H(:,end)>0));
mmr1 = min(mr1);
mmr2 = min(mr2);
maskRow = min(mmr1,mmr2);
maskRectString = strcat(num2str(maskRow),'_1_', num2str(size(tmpI,1)-maskRow), '_',num2str(size(tmpI,2)) );

end
