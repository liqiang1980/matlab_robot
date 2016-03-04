% integrator for integrating value in the vector together 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% exploration action in order to estimate the normal direction
% sponsered by DFG spp-1527: autonmous learning
% author: Qiang Li, Bielefeld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load dis_set;
s=size(dis_set);
for i = 1:s(2)-1
    ss(i) = dis_set(i)+dis_set(i+1);
end