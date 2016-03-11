% orientation frame rotation test while old frame is known and new updated
% z is also  known. this routine provide the test how to compute and
% visualize the new orientation frame.
% See also 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sponsered by DFG spp-1527: autonmous learning
% author: Qiang Li, Bielefeld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

startf = trotz(rot_value(3))*troty(rot_value(2))*trotx(rot_value(1));
trplot(startf);
hold on;
nv_start = startf(1:3,3);
nv_end_tmp = rand(3,1);
nv_end = nv_end_tmp/norm(nv_end_tmp);
vectarrow(zeros(3,1),nv_end);
angle = atan2(norm(cross(nv_start,nv_end)), dot(nv_start,nv_end))
rotate_axis = cross(nv_start,nv_end)/norm(cross(nv_start,nv_end));
axis_angle = [rotate_axis; angle];
hold on;
vectarrow(zeros(3,1),rotate_axis);
hold on;
endf=eye(4);
endf(1:3,1:3) = startf(1:3,1:3) * vrrotvec2mat(axis_angle);
trplot(endf,'color','r');

angle2 = atan2(norm(cross(startf(1:3,2),endf(1:3,2))), dot(startf(1:3,2),endf(1:3,2)))