% this is the code to generate a small sphere in order to analog the tool
% interact with
% function drawsphere(c,r) is function to update
%
% c: center of the sphere,array 3*1
% r: radius of the sphere
% See also  
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% exploration action in order to estimate the normal direction
% sponsered by DFG spp-1527: autonmous learning
% author: Qiang Li, Bielefeld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function drawsphere(c,r)
[x,y,z] = sphere(50);
x = x*r + c(1);
y = y*r + c(2);
z = z*r + c(3);
surface(x,y,z);