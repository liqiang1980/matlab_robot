% this is the code to compute the slope and intercept of points set
% function [k,b,sign_k,deltax,atan2_k,k2] = regression_2d(ps) 
% k: slope for the atan(deltay/deltax)
% b: 
% deltay:y(endpoint) - y(startpoint)
% deltax:x(endpoint) - x(startpoint)
% atan2_k: atan2(deltay,deltax);
% k2: pca-similar method to computed slope
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sponsered by DFG spp-1527: autonmous learning
% author: Qiang Li, Bielefeld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [k,b,deltay,deltax,atan2_k,k2] = regression_2d(ps)
data_set = size(ps);
sum_x = 0;
sum_y = 0;
sum_xy = 0;
sum_x2 = 0;
for i = 1:data_set(1)
    sum_x = sum_x + ps(i,1); 
    sum_y = sum_y + ps(i,2); 
    sum_xy = sum_xy + ps(i,1) * ps(i,2); 
    sum_x2 = sum_x2 + ps(i,1) * ps(i,1);
end
mean_x = sum_x/data_set(1);
mean_y = sum_y/data_set(1);

varx = sum_x2 - sum_x * mean_x;
cov = sum_xy - sum_x * mean_y;

deltay = ps(data_set(1),2) - ps(1,2);
deltax = ps(data_set(1),1) - ps(1,1);

k2 = atan(cov / varx)
k = atan(deltay/deltax);
b = mean_y - k * mean_x;

atan2_k = atan2(deltay,deltax)