% Test rotation angle estimation between two 2d frames while points clouds
% given in the two frames.
% See also 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sponsered by DFG spp-1527: autonomous learning
% author: Qiang Li, Bielefeld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

data_num = 6;
source_p = randn(data_num,2);
target_p = zeros(data_num,2);
target_p_3 = zeros(3,data_num);
theta = 0.8;
tx = 0.01;
ty = 0.03;
r_matrix = [cos(theta) sin(theta) tx;...
   -sin(theta) cos(theta) ty;...
   0 0 1]
for i = 1:data_num
    target_p_3(:,i) = r_matrix * [source_p(i,:) 1]';
    target_p(i,1) =  target_p_3(1,i)/target_p_3(3,i)+0.005*randn;
    target_p(i,2) =  target_p_3(2,i)/target_p_3(3,i)+0.005*randn;
end

h_matrix = est_rotatematrix(source_p,target_p)
