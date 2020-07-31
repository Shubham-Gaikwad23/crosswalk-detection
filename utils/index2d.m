function d = index2d(idx, resolution, maxd)
%     d = idx*resolution-maxd-1;
    d = (idx*resolution-maxd) - resolution/2;
%     binTop = (idx+1)*resolution-maxd
%     d = (binTop-binBottom)/2;
end