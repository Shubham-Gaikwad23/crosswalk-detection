function linesHist(lines,res)

%find rho < 0
% ind = find(lines(:,2)<0);

% lines(ind,2) = -lines(ind,2);
lines(:,3) = mod(lines(:,3),180);

% ind0 = find(lines(:,2)==0);
% lines(ind0,2) = 360;

figure, hist(lines(:,3),360/res);

end
