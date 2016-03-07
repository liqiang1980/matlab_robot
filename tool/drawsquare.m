% this is the code to generate a patch plane in order to analog the tool
% surface
% function drawsquare(tm,size,color) is to draw a transformed square by tm,
% the square is colored.
%
% tm: homegeneous matrix in order to transform square from original posture
% size: length of the side, scalar
% color: patch color, array 3*1
% See also  
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sponsered by DFG spp-1527: autonmous learning
% author: Qiang Li, Bielefeld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function drawsquare(tm,size,color)
P1 = [-size/2,size/2,0];
P2 = [-size/2,-size/2,0];
P3 = [size/2,-size/2,0];
P4 = [size/2,size/2,0];
P1_new = tm * [P1,1]';
P2_new = tm * [P2,1]';
P3_new = tm * [P3,1]';
P4_new = tm * [P4,1]';
P = [P1_new(1:3)';P2_new(1:3)';P3_new(1:3)';P4_new(1:3)'];
x = P(:,1)';
y = P(:,2)';
z = P(:,3)';
patch(x,y,z,color)
xlabel('X')
ylabel('Y')
zlabel('Z')
view(3)
% rotate3d