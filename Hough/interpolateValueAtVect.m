function [val, idx] = interpolateValueAtVect(x,y,I, interpolate)
val = zeros(length(x),1);
idx = find(x >=1 & x <= size(I,2) & y >= 1 & y <= size(I,1));

if interpolate
    hfrac = x(idx) - floor(x(idx)); % Fractional offset of xoff relative to integer location
    vfrac = y(idx) - floor(y(idx)); % Fractional offset of yoff relative to integer location
    
    
    fx = floor(x(idx));          % Get integer pixel locations that surround location x,y
    cx = ceil(x(idx));
    fy = floor(y(idx));
    cy = ceil(y(idx));
    tl = I(sub2ind(size(I),fy,fx));    % Value at top left integer pixel location.
    tr = I(sub2ind(size(I),fy,cx));    % top right
    bl = I(sub2ind(size(I),cy,fx));    % bottom left
    br = I(sub2ind(size(I),cy,cx));    % bottom right
    
    upperavg = tl + hfrac .* (tr - tl);  % Now use bilinear interpolation to
    loweravg = bl + hfrac .* (br - bl);  % estimate value at x,y
    val(idx) = upperavg + vfrac .* (loweravg - upperavg);
else
    val(idx) = I(sub2ind(size(I),round(y(idx)),round(x(idx))));
end

end
