function thetas = getThetaRangeVec(theta, delta, resolution)

if resolution > delta
    delta = 1;
end

thetas = zeros(length(theta),2*floor(delta/resolution)+1);
for i = 1 : length(theta)
%     theta(i)
    thetasLeft = flip(theta(i) : -resolution : theta(i)-delta,2);
    thetasLeft(thetasLeft<0) = 360 + thetasLeft(thetasLeft<0);
    thetasRight = theta(i)+resolution : resolution : theta(i)+delta;
    thetasRight(thetasRight>=360) = thetasRight(thetasRight>=360) - 360;
    thetas(i,:) = [thetasLeft thetasRight];
end

