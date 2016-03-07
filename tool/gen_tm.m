% this is the code to generate a random homogeneous matrix
% function tm = gen_tm() 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sponsered by DFG spp-1527: autonmous learning
% author: Qiang Li, Bielefeld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function tm = gen_tm()
T1 = rpy2tr(rand,rand,rand);
T2 = transl(rand,rand,rand);
tm = T1 * T2;
