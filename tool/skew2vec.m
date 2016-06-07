% Generate a skew matrix from vector
%
% X = vec2skew(x)
% See also 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sponsered by DFG spp-1527: autonmous learning
% author: Qiang Li, Bielefeld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function x = skew2vec(X)

x = [X(3,2);X(1,3);X(2,1)];
end