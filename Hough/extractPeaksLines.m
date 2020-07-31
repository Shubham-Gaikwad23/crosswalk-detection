function lines = extractPeaksLines(H,minPeak, resolution_theta,...
    resolution_d, maxd, radius_step, minradius, numRadius)
% EXTRACTPEAKSLINES: Extract lines associated to Hough peaks
% OUTPUT:
%   lines - retrieved lines in the form lines = ( : , [peak value,
%   rho, theta, width])

lines = [];
for w = 0 : numRadius-1
    Hw = H(:,:,w+1);
    if minPeak > 0
        peaks = houghpeaks_Floats(Hw(:,:), 50,'Threshold',minPeak,'NHoodSize', [9 9]);
    else
        peaks = houghpeaks_Floats(Hw(:,:),50, 'NHoodSize', [8 8]);
    end
    for l = 1 : size(peaks,1)
        d_idx = peaks(l,1);
        t_idx = peaks(l,2);

        theta = (t_idx-1)*resolution_theta;
        d = index2d(d_idx, resolution_d, maxd);
        lines(end+1,:) = [Hw(d_idx,t_idx) d theta minradius+w*radius_step];
    end
end

end
