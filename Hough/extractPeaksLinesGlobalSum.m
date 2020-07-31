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

