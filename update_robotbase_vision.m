% this is the code to compute the arm base posture
% using the vision-based calibration homogeneous matrix(HM) representation
% input arm_id: 0 right arm, 1 left arm
% author: Qiang Li, Bielefeld

function update_robotbase_vision(arm_id)

cor_hm = eye(4); 
% 0 is right arm
if(arm_id == 0)
    cad_base_hm =[0.433466   -0.250194    0.865743      0.0823;
        0.499272    0.866445 0.000418414       0.897;
        -0.750223    0.432059    0.500489      0.2975;
        0           0           0           1];
    
    cor_hm=[ 0.999838 0.0157348 0.00869609 -0.0134792;
        -0.0157843 0.999859 0.00565991 0.00670717;
        -0.00860581 -0.00579626 0.999946 0.00319055;
        0 0 0 1 ];
end
% 1 is left arm
if (arm_id == 1)
    cad_base_hm =[0.433466   -0.250194    0.865743      0.0823;
        0.499272    0.866445 0.000418414       0.897;
        -0.750223    0.432059    0.500489      0.2975;
        0           0           0           1];
    
    cor_hm=[ 0.999838 0.0157348 0.00869609 -0.0134792;
        -0.0157843 0.999859 0.00565991 0.00670717;
        -0.00860581 -0.00579626 0.999946 0.00319055;
        0 0 0 1 ];
end

vis_base_hm = cor_hm * cad_base_hm;

tr2rpy(vis_base_hm,'zyx')

[R,t] = tr2rt(vis_base_hm)


