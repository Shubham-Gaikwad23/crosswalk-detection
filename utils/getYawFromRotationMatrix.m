function yaw = getYawFromRotationMatrix(R)


% Call the 3x3 rotation matrix R (we are assuming that it has already been defined)
% Let's call the horizontal plane H
N_H = R(2,1:3) + 0; %mag. north, i.e., magnetic field projected on H
N_H = N_H / norm(N_H);
up_R = R(3,1:3) + 0;
c_H = project_vec([0.,0.,-1.], up_R); %#camera line of sight projected on H
c_H = c_H / norm(c_H);

% yaw = sign(dot(cross(c_H, N_H),up_R))*acos(dot(N_H, c_H))*180/pi;
yaw = sign(dot(cross(c_H, N_H),up_R))*acos(dot(N_H, c_H));


end


function p_hat = project_vec(v,n)
    n_hat = n/norm(n);
	p_hat = v - dot(v,n_hat)*n_hat;
end

