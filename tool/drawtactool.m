% this is the code to draw tactile tool
% function drawstool(robottm,tooltm,size,color) 
%
% robottm: robot end effector homegeneous matrix 
% tooltm: robot end effector homegeneous matrix 
% size: length of the side, scalar
% color: patch color, array 3*1
% See also  
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sponsered by DFG spp-1527: autonmous learning
% author: Qiang Li, Bielefeld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function drawtactool(robottm,tooltm,size,color) 
P1 = robottm(1:3,4)';
P2 = tooltm(1:3,4)';
pts = [P1; P2];
%bar represent the handle of the tactile tool
% plot3(pts(:,1), pts(:,2), pts(:,3),'LineWidth',5);
%square represent the myrmex sensor on the tactile tool
drawsquare(tooltm,size,color);