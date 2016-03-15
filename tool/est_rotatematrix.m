function hmatrix = est_rotatematrix(sourcePoints,targetPoints)
% code source
% http://stackoverflow.com/questions/23619269/calculating-translation-value-and-rotation-angle-of-a-rotated-2d-image
% http://www.comp.nus.edu.sg/~cs4243/lecture/camera.pdf
numPoints = size(sourcePoints, 1);

%// Cast data to double to be sure
sourcePoints = double(sourcePoints);
targetPoints = double(targetPoints);

%//Extract relevant data
xSource = sourcePoints(:,1);
ySource = sourcePoints(:,2);
xTarget = targetPoints(:,1);
yTarget = targetPoints(:,2);

%//Create helper vectors
vec0 = zeros(numPoints, 1);
vec1 = ones(numPoints, 1);

xSourcexTarget = -xSource.*xTarget;
ySourcexTarget = -ySource.*xTarget;
xSourceyTarget = -xSource.*yTarget;
ySourceyTarget = -ySource.*yTarget;

%//Build matrix
A = [xSource ySource vec1 vec0 vec0 vec0 xSourcexTarget ySourcexTarget; ...
    vec0 vec0 vec0 xSource ySource vec1 xSourceyTarget ySourceyTarget];

%//Build RHS vector
b = [xTarget; yTarget];

%//Solve homography by least squares
h = A \ b;

%// Reshape to a 3 x 3 matrix (optional)
%// Must transpose as reshape is performed
%// in column major format
h(9) = 1; %// Add in that h33 is 1 before we reshape
hmatrix = reshape(h, 3, 3)';