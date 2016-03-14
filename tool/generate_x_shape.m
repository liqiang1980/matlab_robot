% this is the code to generate a closed x shape for the moving trajectory
% function vel = generate_x_shape(j)
%
% j: input index
% vel: point velocity
% See also  
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sponsered by DFG spp-1527: autonmous learning
% author: Qiang Li, Bielefeld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function vel = generate_x_shape(j)
vel = zeros(3,1);
period = 180;
mod_j = mod(j,period);
if((mod_j<=20)&&(mod_j)>0)
    vel(1)= 0.001;vel(2) = 0.001;
elseif((mod_j<=60)&&(mod_j)>20)
    vel(1)= 0;vel(2) = -0.001;
elseif((mod_j<=120)&&(mod_j)>60)
    vel(1)= -0.001;vel(2) = 0.001;
elseif((mod_j<=160)&&(mod_j)>120)
    vel(1)= 0;vel(2) = -0.001;
else
    vel(1)= 0.001;vel(2) = 0.001;
end
