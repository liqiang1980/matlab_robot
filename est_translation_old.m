% this is the code to estimate translation assume linear velocity
% and rotation velocity are measurable.
% from "online contact point estimation for uncalibrated tool use"
% published by IRCA, Yiannis Karayiannidis
% author: Qiang Li, Bielefeld



%generate the roation data given a
T_init = trotx(0) * troty(0) * trotz(0);
trplot(T_init, 'frame', 'A');
hold on;
link_value = [0.1,0.3,0.3];
% used for test the rotation relation of toolbox
% T1 = troty(0.2)* T_init;
% trplot(T1, 'frame', 'B','color','r');
% T2 = trotx(0.2)*T1;
% trplot(T2, 'frame', 'C','color','g');
T_eff_start = transl(link_value) * T_init;
trplot(T_eff_start, 'frame', 'B','color','r');

T_eff_end_last = eye(4);
Gama_r = 300*eye(3);
L_r = zeros(3);
L_r_dot = zeros(3);
c_r = zeros(3,1);
c_r_dot = zeros(3,1);
beta_r = 0.99;
est_trans = rand(3,1);
est_trans_dot = zeros(3,1);


%rotation angle stimulation.

sample_num = 500;
ux = idinput(sample_num,'prbs');
uy = idinput(sample_num,'prbs');
uz = idinput(sample_num,'prbs');
    for j =1:1:sample_num
        %round global x
%          T_eff_end_cur = r2t(rotx(0.1*sin(j*0.01)))*r2t(roty(0.1*sin(j*0.01))) *r2t(rotz(0.1*sin(j*0.01))) * T_eff_start;
          T_eff_end_cur = r2t(rotx(0.05*rand)) *r2t(roty(0.05*rand))*r2t(rotz(0.05*rand)) * T_eff_start;
%         T_eff_end_cur = r2t(rotx(0.005*ux(j))) *r2t(roty(0.005*uy(j)))*r2t(rotz(0.005*uz(j))) * T_eff_start;
        %around local x
        %     R_eff_start = t2r(T_eff_start);
        %     R_eff_end = rotx(j*0.01) * R_eff_start;
        %     T_eff_end = transl(link_value) * r2t(R_eff_end);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%estimation alg part%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%mls94 p.72 electronic version,eq. 2.53;
        %estimate the rotation velocity
        omiga_skmatrix = (t2r(T_eff_end_cur) - t2r(T_eff_end_last)) *(t2r(T_eff_end_cur))';
%         vel = (-1)*(t2r(T_eff_end_cur) - t2r(T_eff_end_last))*(t2r(T_eff_end_cur))'*...
%             T_eff_end_cur(1:3,4) +(T_eff_end_cur(1:3,4)-T_eff_end_last(1:3,4))
        omiga_vec = [omiga_skmatrix(3,2);omiga_skmatrix(1,3);omiga_skmatrix(2,1)];
        omiga_vec_est(j,:) = omiga_vec;
        vel = (T_eff_end_cur(1:3,4)-T_eff_end_last(1:3,4)+0.005*rand(3,1));

        L_r_dot = (-1)*beta_r*L_r-omiga_skmatrix*omiga_skmatrix;
        c_r_dot = (-1)*beta_r*c_r+omiga_skmatrix*(-1)*vel;
        est_trans_dot = (-1)*Gama_r*(L_r*est_trans-c_r);
        L_r = L_r + L_r_dot;
        c_r = c_r + c_r_dot;
        est_trans = est_trans + est_trans_dot
        est(j,1:3) = est_trans;
        est(j,4) = norm(cross(omiga_vec,est_trans)-vel);
        T_eff_end_last = T_eff_end_cur;
        %visualization
%         tranimate(T_eff_start,T_eff_end_cur);
    end
figure(2)
subplot(4,1,1);
plot(2:sample_num,est(2:sample_num,1));
subplot(4,1,2);
plot(2:sample_num,est(2:sample_num,2));
subplot(4,1,3);
plot(2:sample_num,est(2:sample_num,3));
subplot(4,1,4);
plot(2:sample_num,est(2:sample_num,4));

figure(3);
subplot(3,1,1);
plot(1:sample_num,omiga_vec_est(1:sample_num,1));
subplot(3,1,2);
plot(1:sample_num,omiga_vec_est(1:sample_num,2));
subplot(3,1,3);
plot(1:sample_num,omiga_vec_est(1:sample_num,3));


