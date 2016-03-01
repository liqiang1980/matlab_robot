% this is the code to generate a orthogonal matrix frame z vector.
% author: Qiang Li, Bielefeld
function RM = CreateRotationByNormal(nv)
z_normalize = nv/norm(nv);
y = zeros(3,1);
y(1) = z_normalize(2);
y(2) = -1 * z_normalize(1);
y(3) = 0;
y_normalize = y/norm(y);
x_normalize = cross(y_normalize,z_normalize);
RM = zeros(3);
RM(:,1) = x_normalize;
RM(:,2) = y_normalize;
RM(:,3) = z_normalize;



