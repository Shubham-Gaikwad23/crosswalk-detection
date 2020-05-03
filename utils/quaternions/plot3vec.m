function plot3vec(R) % assumes input is N x 3 matrix
W = [];
for r = 1 : length(R)
A = R{r};
[V,D] = eig(A);
[ignore,ix] = min(abs(diag(D)-1));
w = V(:,ix);
t = [A(3,2)-A(2,3),A(1,3)-A(3,1),A(2,1)-A(1,2)];
theta = atan2(t*w,trace(A)-1);
if theta<0, theta = -theta; w = -w; end
W(end+1,:) = w/theta;
end
 r = reshape(W,1,[]); % put data all in one row
 r = [zeros(size(r));r]; % interleave 0's in between the data
 r = reshape(r,[],3); % reshape back to 2N x 3
 plot3(r(:,1),r(:,2),r(:,3),'-o'); % plot it
 grid on; % turn grid on

end