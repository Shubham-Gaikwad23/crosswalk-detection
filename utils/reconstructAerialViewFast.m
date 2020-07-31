function [J, Jint, Mask, Mapping] = reconstructAerialViewFast(Ic, focal, ...
    camHeight, upVector, azimuth, meterXpixel, Urange, Vrange)
%RECONSTRUCTAERIALVIEW reconstructs an aerial view of a given image
%
% INPUT
%   I - input image (grayscale)
%   focal - focal length of the camera
%   camHeight - height of the camera wrt the ground plane
%   upVector - accelerometer sensor data
%   azimuth - angle (in degrees) of the camera wrt the magnetic North
%   meterXpixel - specify the reconstruction image resolution, given in
%   meters per pixel.
%
% OUTPUT
%   J - reconstructed aerial view
%   Jint - interpolated aerial view
%   Eint - edges of the aerial view projected from the original image edges
%   Mask - binary mask of the pixel below the horizon
%   Mapping - mapping from the pixel in the aerial view to the original
%   image

nHat = upVector/norm(upVector);
k = -camHeight;

p0 = k*nHat;
cHat = [0 0 -1];

bHat = cHat - dot(nHat,cHat)*nHat;
bHat = bHat / norm(bHat);
aHat = cross(bHat,nHat);

u2s = -Urange : meterXpixel : Urange;
v2s = -Vrange : meterXpixel : Vrange;

J = zeros(length(v2s),length(u2s));

I = rgb2gray(Ic);
[h, w] = size(I);

Mask = zeros(length(v2s),length(u2s));
Rot = rotationMatrix2D(azimuth);


Mapping = [];
% ind_u2 = 0;

[u2, v2] = meshgrid(u2s,v2s);
u2 = reshape(u2,length(u2s)*length(v2s),1);
v2 = reshape(v2,length(v2s)*length(u2s),1);

[ind_u2, ind_v2] = meshgrid(1:length(u2s),1:length(v2s));
ind_u2 = reshape(ind_u2,length(u2s)*length(v2s),1);
ind_v2 = reshape(ind_v2,length(u2s)*length(v2s),1);
uv3 = Rot * [u2'; v2'];
pp0 = repmat(p0,size(uv3,2),1);
R = pp0 + uv3(1,:)'*aHat + uv3(2,:)'*bHat;
u = focal(1)*R(:,1)./R(:,3);
v = focal(2)*R(:,2)./R(:,3);
i = u + h/2.;
j = v + w/2.;
i_rnd = (round(i));
j_rnd = (round(j));

idx = (i_rnd>0 & i_rnd<=h & j_rnd>0 & j_rnd<=w);
idx2 = (uv3(2,:)>0);
idx3 = idx.*idx2';
idx = find(idx3>0);

r = length(v2s)-ind_v2(idx)+1;
c = ind_u2(idx);
jIdx = sub2ind(size(J),r,c);

ri = i_rnd(idx);
ci = j_rnd(idx);
iIdx = sub2ind(size(I),ri,ci);

Vmask = uv3(2,:)>0;

J(jIdx) = I(iIdx);

Mask(jIdx) = 1;
Mapping(jIdx,:) = [i(idx) j(idx)];
I = real(I);

Jint = interp2(double(real(I)),j,i,'cubic');
Jint = Jint .* double(Vmask)';
Jint = (imrotate(reshape(Jint,size(J)),180));
Jint = flip(Jint,2);


end

