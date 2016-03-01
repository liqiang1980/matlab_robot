% this is the code to compute the right arm base posture
% using the tactile-based calibration homogeneous matrix(HM) representation
% input 
% (1)left arm base HM
% (2)transform HM
% author: Qiang Li, Bielefeld

%left_hm can comes from the cad model or vision calibration result.
left_hm = [-0.433466   -0.250194   -0.865743     -0.0823;
   0.499272   -0.866445 0.000418414       0.897;
  -0.750223   -0.432059    0.500489      0.2975;
          0           0           0           1]
transform_hm = [0.623932 -0.206093 -0.753813 -0.0697187
 0.228809  -0.87415  0.428379 0.034476;
-0.747231 -0.439759 -0.498254 -0.143885;
0 0 0 1]

right_hm =  left_hm * inv(transform_hm);

tr2rpy(right_hm,'zyx')

[R,t] = tr2rt(right_hm)


