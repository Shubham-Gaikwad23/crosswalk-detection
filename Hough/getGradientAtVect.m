function [g, a, y, x, idx] = getGradientAtVect(G, Gdir, col, row, angle, radius)
%returns the interpolated G value of the point that has distance radius from
%the point x,y along the direction specified by angle.



g = zeros(length(col),1);
a= angle;
xoff = radius*cosd(angle);   % x and y offset of points at specified radius and angle
yoff = radius*sind(angle);   % from each reference position.

hfrac = xoff - floor(xoff); % Fractional offset of xoff relative to integer location
vfrac = yoff - floor(yoff); % Fractional offset of yoff relative to integer location

x = col + xoff;     % x, y location on one side of the point in question
y = row - yoff;

idx = find(x>2 & x<size(G,2)-2 & y > 2 & y < size(G,1)-2);

fx = floor(x(idx));          % Get integer pixel locations that surround location x,y
cx = ceil(x(idx));
fy = floor(y(idx));
cy = ceil(y(idx));
tl = G(sub2ind(size(G),fy,fx));    % Value at top left integer pixel location.
tr = G(sub2ind(size(G),fy,cx));    % top right
bl = G(sub2ind(size(G),cy,fx));    % bottom left
br = G(sub2ind(size(G),cy,cx));    % bottom right

upperavg = tl + hfrac(idx) .* (tr - tl);  % Now use bilinear interpolation to
loweravg = bl + hfrac(idx) .* (br - bl);  % estimate value at x,y
g(idx) = upperavg + vfrac(idx) .* (loweravg - upperavg);

tl = Gdir(sub2ind(size(Gdir),fy,fx));    % Value at top left integer pixel location.
tr = Gdir(sub2ind(size(Gdir),fy,cx));    % top right
bl = Gdir(sub2ind(size(Gdir),cy,fx));    % bottom left
br = Gdir(sub2ind(size(Gdir),cy,cx));    % bottom right

upperavg = tl + hfrac(idx) .* (tr - tl);  % Now use bilinear interpolation to
loweravg = bl + hfrac(idx) .* (br - bl);  % estimate value at x,y
a(idx) = upperavg + vfrac(idx) .* (loweravg - upperavg);


end
