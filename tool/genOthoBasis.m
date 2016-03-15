%creat a othorgnal basis from one arbitary normalized vector
%v_in is array of 3*1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sponsered by DFG spp-1527: autonmous learning
% author: Qiang Li, Bielefeld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function v_out = genOthoBasis(v_in)
x=v_in(:).'/norm(v_in);
yz=null(x).';
v_out=[yz;x]';  %The rows of this matrix are the axes
