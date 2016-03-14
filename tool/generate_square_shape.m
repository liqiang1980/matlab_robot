% this is the code to generate a square shape for the moving trajectory
% function vel = generate_square_shape(j,scale)
%
% j: input index
% vel: point velocity
% scale: used for control the velocity
% See also  
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sponsered by DFG spp-1527: autonmous learning
% author: Qiang Li, Bielefeld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function vel = generate_square_shape(j,scale)
vel = zeros(3,1);
period = 4;
mod_j = mod(j,period);
if((mod_j<=1)&&(mod_j)>0)
    vel(1)= scale;vel(2) = 0;
elseif((mod_j<=2)&&(mod_j)>1)
    vel(1)= 0;vel(2) = -scale;
elseif((mod_j<=3)&&(mod_j)>2)
    vel(1)= -scale;vel(2) = 0;
else
    vel(1)= 0;vel(2) = scale;
end
