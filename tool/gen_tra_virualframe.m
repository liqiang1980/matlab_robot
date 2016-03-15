% this is the code to generate intermedia tajectory in the a square shape
% function ct_V = gen_tra_virualframe(j,k,scale)
%
% j: input index
% ct_V
% scale: is the longth of every step in normal direction learning, refer to
% the function update_ct_surf--em = 5;
% See also  generate_square_shape
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sponsered by DFG spp-1527: autonmous learning
% author: Qiang Li, Bielefeld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function p = gen_tra_virualframe(j,k,scale,sample_num)
p = zeros(2,1);
period = 4;
mod_j = mod(j,period);
if((mod_j<=1)&&(mod_j)>0)
    p(1)= k*scale/sample_num;p(2) = 0;
elseif((mod_j<=2)&&(mod_j)>1)
    p(1)= 0;p(2) = -k*scale/sample_num;
elseif((mod_j<=3)&&(mod_j)>2)
    p(1)= -k*scale/sample_num;p(2) = 0;
else
    p(1)= 0;p(2) = k*scale/sample_num;
end
