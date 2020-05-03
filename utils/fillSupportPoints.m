% Copyright (c) 2015, The Smith-Kettlewell Eye Research Institute
% All rights reserved.
% 
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
%     * Redistributions of source code must retain the above copyright
%       notice, this list of conditions and the following disclaimer.
%     * Redistributions in binary form must reproduce the above copyright
%       notice, this list of conditions and the following disclaimer in the
%       documentation and/or other materials provided with the distribution.
%     * Neither the name of the The Smith-Kettlewell Eye Research Institute nor
%       the names of its contributors may be used to endorse or promote products
%       derived from this software without specific prior written permission.
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
% ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
% WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
% DISCLAIMED. IN NO EVENT SHALL THE SMITH-KETTLEWELL EYE RESEARCH INSTITUTE BE
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
% DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
% LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
% ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
% (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
% SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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