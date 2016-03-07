% this is the test code to visualize the plan along the same surface but
% differnt position
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sponsered by DFG spp-1527: autonmous learning
% author: Qiang Li, Bielefeld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
TR = rpy2tr(0.2,0.3,0.4);
TS = transl(0.1,0.2,0.3);
T = TR * TS;
color = [0.1,0.4,0.5];
drawsphere(T(1:3,4),0.005);
hold on;
for i = 1:10
    local = 0.01*rand(3,1);
    local(3) = 0;
    gl = T(1:3,1:3) * local;
    T(1:3,4) = T(1:3,4) + gl;
    drawsquare(T,0.2,color);
end