function [d_idx, diff] = d2index(d, resolution_d, maxd)
 
    d_idx = floor( (d + maxd)/resolution_d)+1;
    diff = (d + maxd) - (resolution_d.*d_idx - resolution_d/2);
end