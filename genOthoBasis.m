%creat a othorgnal basis from one arbitary normalized vector
function v_out = genOthoBasis(v_in);
x=v_in(:).'/norm(v_in)
yz=null(x).';
v_out=[x;yz];  %The rows of this matrix are the axes
