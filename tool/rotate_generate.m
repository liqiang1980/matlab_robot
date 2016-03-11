% orientation frame rotation test while old frame is known and new updated
% z is also  known. this routine provide the test how to compute and
% visualize the new orientation frame.
%
%
% endf = rotate_generate(startf,nv)
% startf: start homogeneous matrix
% nv: new orientation matrix normal direction(z axis)
% endf: final orientation matrix
% See also 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sponsered by DFG spp-1527: autonmous learning
% author: Qiang Li, Bielefeld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function endf = rotate_generate(startf,nv)
% trplot(startf);
% hold on;
nv_start = startf(1:3,3);
nv_end_tmp = nv;
nv_end = nv_end_tmp/norm(nv_end_tmp);
% vectarrow(zeros(3,1),nv_end);
angle = atan2(norm(cross(nv_start,nv_end)), dot(nv_start,nv_end));
rotate_axis = cross(nv_start,nv_end)/norm(cross(nv_start,nv_end));
axis_angle = [rotate_axis; angle];
% hold on;
% vectarrow(zeros(3,1),rotate_axis);
% hold on;
endf=eye(4);
endf(1:3,1:3) = startf(1:3,1:3) * vrrotvec2mat(axis_angle);
endf(1:3,4) = startf(1:3,4);
% trplot(endf,'color','r');
