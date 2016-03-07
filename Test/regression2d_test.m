% this is the test code for regression_2d in tool folder.
% function [k,b,sign_k] = regression_2d(ps) 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sponsered by DFG spp-1527: autonmous learning
% author: Qiang Li, Bielefeld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

r0 = [0,-1]';
k1 = [0.01;0.999];
k = k1/norm(k1);

% k = [0.707;0.707];
r = zeros(2,100);
ind = 1;
for t = 0:0.01:2
    r(:,ind) = r0+ t*k;
    ind = ind + 1;
end
line(r(1,:),r(2,:));

[k,b,deltay,deltax,atan2_k,k2] = regression_2d(r')