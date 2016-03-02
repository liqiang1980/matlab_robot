% this is the code to let the robot learn the usage of tool in a
% exploration way. sponsered by DFG spp-1527: autonmous learning
% author: Qiang Li, Bielefeld

close all;

%load robot model
kuka_robot = loadrobot('kukalwr');
% %     visualization of kuka_lwr at the initialized pose
Q = rand(1,7);
%model the tactool frame
T_init = kuka_robot.fkine(Q);
link_value = rand(3,1);
rot_value = [0.2,0.5,0.3];
tool_rotate = trotz(rot_value(3))*troty(rot_value(2))*trotx(rot_value(1));
tool_translate = transl(link_value);
tool_transform = tool_rotate * tool_translate ;
T_eff_start = T_init*tool_transform;
virtual_angle = 6;
rot_tm = rpy2tr(0,0,virtual_angle);
T_eff_start_virtual = T_eff_start*rot_tm;

virtual_x = 6;
virtual_y = 6;
virtual_z = 6;
rot_tm = rpy2tr(virtual_x,virtual_y,virtual_z);
T_eff_start_noise = T_eff_start*rot_tm;
trplot(T_eff_start_noise, 'frame', 'N');
hold on;


T_robot_eff_end_last = eye(4);
T_tool_eff_end_last = eye(4);
Gama_r = 30*eye(3);
L_r = zeros(3);
L_r_dot = zeros(3);
c_r = zeros(3,1);
c_r_dot = zeros(3,1);
beta_r = 0.99;
est_trans = zeros(3,1);
est_trans_dot = zeros(3,1);
P_bar = zeros(3);
L_n = zeros(3);
L_n_dot = zeros(3);
beta_n = 0.99;
gamma_n = 300;
n_hat = zeros(3,1);
n_hat_dot = zeros(3,1);

%sliding exploration in order to estimate the rotation angle
%if the tactool slides along x axis-positive, the trajectory we get from
%tactile sensor is along x axis-negtive, in simulation, the negtive x axis
%is known which is Rv_r*[-1,0,0]; where Rv_r is transformatrix from virtual
%frame to the real tactile sensor frame.
rot_tm = rpy2tr(0,0,virtual_angle);
vec = rot_tm(1:3,1:3) * [-1;0;0];
angle = atan2(vec(2),vec(1));

if(sign(angle) == 1 )
    if((vec(1)>0)&&(vec(2)>0))
        gama = atan(vec(2)/vec(1))+pi;
    else
        gama = atan(vec(2)/vec(1));
    end
else
    if((vec(1)>0)&&(vec(2)<0))
        gama = atan(vec(2)/vec(1))+pi;
    else
        gama = atan(vec(2)/vec(1));
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%exploration action in order to estimate the normal direction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sample_num = 1500;
n_hat = T_eff_start_noise(1:3,3)
T_eff_start(1:3,3)


for j =1:1:sample_num
    %compute the current robot and tool configure
    T_robot_eff_end_cur = kuka_robot.fkine(Q);
    T_tool_eff_end_cur = T_robot_eff_end_cur*tool_transform;
%     trplot(T_eff_cur, 'frame', 'A');
%     trplot(T_tool_cur, 'frame', 'B','color','r');
    %desired velocity in the local tool end-effector frame
    p_e_dot_local = 0.005*rand(3,1);
    p_e_dot_local(3) = 0;
    p_e_dot = T_tool_eff_end_cur(1:3,1:3) * p_e_dot_local;
    %from the desired linear velocity computing the joint angle rate
    Jac = kuka_robot.jacob0(Q);
    q_dot = pinv(Jac)*[p_e_dot;0;0;0];
    % draw tool with line between two 3d points
    P1 = T_eff_cur(1:3,4)';
    P2 = T_tool_cur(1:3,4)';
    pts = [P1; P2];
    plot3(pts(:,1), pts(:,2), pts(:,3),'LineWidth',5);
%     kuka_robot.plot(Q,'workspace', [-3 3 -3 3 -3 3]);
    hold on;
    
    P_bar = eye(3)-n_hat*n_hat';
    n_hat_dot = -1*gamma_n*P_bar*L_n*n_hat;
    L_n_dot = -beta_n*L_n+(1/(1+norm(p_e_dot)^2))*p_e_dot*p_e_dot';
    n_hat = n_hat+n_hat_dot;
    L_n = L_n + L_n_dot;
    if(mod(sample_num,10) == 0)
%         disp('updated normal dir');
%         n_hat
        %estimated normal direction
        est_nv_start = T_tool_cur(1:3,4)';
        est_nv_end = est_nv_start+n_hat';
        pts = [est_nv_start; est_nv_end];
        plot3(pts(:,1), pts(:,2), pts(:,3),'b-');
        hold on;
        %estimated normal direction
        nv_start = T_tool_cur(1:3,4)';
        nv_end = T_tool_cur(1:3,3)';
        pts = [nv_start; nv_end];
        plot3(pts(:,1), pts(:,2), pts(:,3),'r-');
        hold on;
        
    end
    %update joint angle
    Q= Q+q_dot';
end
disp('updated normal dir');
        n_hat

%rotation action in order to estimate the translation.

sample_num = 300;
    for j =1:1:sample_num
        %kuka_lwr generate the exploration action using its end-effector
        %rotation
        rotate_actx = r2t(rotz(0.1*rand));
        rotate_acty = r2t(roty(0.1*rand));
        rotate_actz = r2t(rotx(0.1*rand));
        T_robot_eff_end_cur = T_init*rotate_actz*rotate_acty*rotate_actx;
        T_tool_eff_end_cur = T_robot_eff_end_cur*tool_transform;
%         trplot(T_robot_eff_end_cur, 'frame', 'C');
%         trplot(T_tool_eff_end_cur, 'frame', 'D');
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%estimation alg part%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%mls94 p.72 electronic version,eq. 2.53;
        %estimate the rotation velocity
        omiga_skmatrix = (t2r(T_robot_eff_end_cur) - t2r(T_robot_eff_end_last)) *(t2r(T_robot_eff_end_cur))';
%         vel = (-1)*(t2r(T_eff_end_cur) - t2r(T_eff_end_last))*(t2r(T_eff_end_cur))'*...
%             T_eff_end_cur(1:3,4) +(T_eff_end_cur(1:3,4)-T_eff_end_last(1:3,4))
        omiga_vec = [omiga_skmatrix(3,2);omiga_skmatrix(1,3);omiga_skmatrix(2,1)];
        omiga_vec_est(j,:) = omiga_vec;
        rotation_rm = T_init*rotate_actx;
%         disp('rotation axis');
%         rotation_axis = rotation_rm(1:3,3)
%         disp('rotation from the estimation')
%         omiga_vec/norm(omiga_vec)
        %compute the linear velocity from the differentiate position of the end-effector of
        %the tool
        vel = (-1)*(T_tool_eff_end_cur(1:3,4)-T_tool_eff_end_last(1:3,4)+0.01*rand(3,1));

        L_r_dot = (-1)*beta_r*L_r-omiga_skmatrix*omiga_skmatrix;
        c_r_dot = (-1)*beta_r*c_r+omiga_skmatrix*vel;
        est_trans_dot = (-1)*Gama_r*(L_r*est_trans-c_r);
        L_r = L_r + L_r_dot;
        c_r = c_r + c_r_dot;
        est_trans = est_trans + est_trans_dot;
        est_trans2=(T_robot_eff_end_cur * tool_rotate)'*[est_trans;0];
        est(j,1:3) = est_trans2(1:3);
        est(j,4) = norm(cross(omiga_vec,est_trans2(1:3))-vel);
        T_robot_eff_end_last = T_robot_eff_end_cur;
        T_tool_eff_end_last = T_tool_eff_end_cur;
        %visualization
%         tranimate(T_eff_start,T_eff_end_cur);
    end
figure(2)
subplot(4,1,1);
plot(200:sample_num,est(200:sample_num,1));
subplot(4,1,2);
plot(200:sample_num,est(200:sample_num,2));
subplot(4,1,3);
plot(200:sample_num,est(200:sample_num,3));
subplot(4,1,4);
plot(200:sample_num,est(200:sample_num,4));
disp('estimated link parameters')
[mean(est(200:sample_num,1)),mean(est(200:sample_num,2)),mean(est(200:sample_num,3))]'

figure(3);
subplot(3,1,1);
plot(1:sample_num,omiga_vec_est(1:sample_num,1));
subplot(3,1,2);
plot(1:sample_num,omiga_vec_est(1:sample_num,2));
subplot(3,1,3);
plot(1:sample_num,omiga_vec_est(1:sample_num,3));

disp('real link parameters')
link_value


