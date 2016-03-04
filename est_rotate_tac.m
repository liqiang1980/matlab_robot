% Estimate the rotation angle of tool frame tagent surface using the
% updated normal direction estimation
%
% rotate_angle = est_rotate_tac() is function to update
% the normal direction of tool given a initialized guess from approaching
% trajectory
% See also 
%
%if the tactool slides along x axis-positive, the trajectory we get from
%tactile sensor is along x axis-negtive, in simulation, the negtive x axis
%is known which is Rv_r*[-1,0,0]; where Rv_r is transformatrix from virtual
%frame to the real tactile sensor frame.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sliding exploration in order to estimate the rotation ang
% sponsered by DFG spp-1527: autonmous learning
% author: Qiang Li, Bielefeld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function rotate_angle = est_rotate_tac(virtual_angle)
rot_tm = rpy2tr(0,0,virtual_angle,'deg');
vec = rot_tm(1:3,1:3) * [-1;0;0];
angle = atan2(vec(2),vec(1));

if(sign(angle) == 1 )
    if((vec(1)>0)&&(vec(2)>0))
        gama = atan(vec(2)/vec(1))+pi;
    else
        gama = atan(vec(2)/vec(1));
    end
else
    if((vec(1)>0)&&(vec(2)<0))
        gama = atan(vec(2)/vec(1))+pi;
    else
        gama = atan(vec(2)/vec(1));
    end
end
rotate_angle = gama;
