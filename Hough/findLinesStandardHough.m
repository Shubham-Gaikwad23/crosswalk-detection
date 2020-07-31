function lines = findLinesStandardHough(I, E, numlines, theta_res, minlen, verbose)

thetarange = -90:theta_res:90-theta_res;
[H, theta, rho] = hough(E, 'Theta', thetarange);



P = houghpeaks(H,numlines);
lines = houghlines(E,theta,rho,P,'FillGap',5,'MinLength',minlen);

if verbose
%     figure, imagesc(H)
    figure, imshow(I, []), hold on
    max_len = 0;
    for k = 1:length(lines)
        xy = [lines(k).point1; lines(k).point2];
        plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
        % Plot beginning and end of lines
        plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
        plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
        % Determine the endpoints of the longest line segment
        len = norm(lines(k).point1 - lines(k).point2);
        if ( len > max_len)
            max_len = len;
            xy_long = xy;
        end
    end
end


