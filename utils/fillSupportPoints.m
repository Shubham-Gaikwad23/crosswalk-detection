function F = fillSupportPoints(J,lines,C,Cw,res_d, res_t, min_w, w_step)
% FILLSUPPORTPOINTS create a binary map in which the voting pixels of the
% lines are set to 1.
%
% INPUT
%       J: composite image
%       LINES: Hough lines
%       C: set of points associated to pairs of (d,theta)
%       CW: set of points mate of C
%       RES_D: d resolution
%       RES_T: theta resolution
%       MIN_W: min width of the stripes
%       W_STEP: step size of the range of widths
%
% OUPUT
%       F: binary mask

F = zeros(size(J));

for l = 1 : size(lines,1)
    line = lines(l,:);
    theta = line(3);
    d = line(2);
    Po = [];
    Pw=[];
    
    maxd = floor(sqrt(size(J,1)^2 + size(J,2)^2));
    theta_idx = floor(theta/res_t)+1;
    d_idx = d2index(d, res_d, maxd);
    if length(line) == 4
        w = line(4);
        if w >0
            w_idx = (w-min_w)/w_step+1;
        else
            w_idx = -w;
        end
        
        Po = C{d_idx,theta_idx,w_idx};
        Pw = (Cw{d_idx,theta_idx,w_idx});
        
    else
        for w = 1 : size(C,3)
            Po = [Po; C{d_idx,theta_idx,w}];
            Pw = [Pw; (Cw{d_idx,theta_idx,w})];
        end
    end
    
    
    Po(:,1) = size(J,1) - Po(:,1);
    Pw(:,1) = size(J,1) - Pw(:,1);
    Po(Po==0) = 1;
    Pw(Pw==0) = 1;
    
    for p = 1 : size(Po,1)
        F(Po(p,1),Po(p,2)) = 255;
        F(round(Pw(p,1)),round(Pw(p,2))) = 255;
    end
end