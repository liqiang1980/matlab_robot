%initialize the parameters
Gama_r = 2000*eye(3);
L_r = zeros(3);
L_r_dot = zeros(3);
c_r = zeros(3,1);
c_r_dot = zeros(3,1);
beta_r = 0.99;
est_trans = zeros(3,1);
est_trans_dot = zeros(3,1);
sample_num = 300;

%load data and pass them by the moving averate filters
a = 1;
b = [1/5 1/5 1/5 1/5 1/5];
omiga_vec_est_filter = filter(b,a,omiga_vec_est);
vel_est_filter = filter(b,a,vel_est);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%drawing area%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(4);
start_p = 10;
for i = 1:3
    subplot(3,1,i)
    plot(omiga_vec_est(start_p:end,i),'r');hold on;
    plot(omiga_vec_est_filter(start_p:end,i),'g');hold on;
    plot(omiga_vec_real(start_p:end,i),'b');hold on;
end

figure(5);
for i = 1:3
    subplot(3,1,i)
    plot(vel_est(start_p:end,i),'r');hold on;
    plot(vel_est_filter(start_p:end,i),'g');hold on;
    plot(vel_real_est(start_p:end,i),'b')
end

for j = start_p:sample_num
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%estimation alg part%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%mls94 p.73 electronic version,eq. 2.53;
    %estimate the rotation velocity in body frame
    omiga_vec = omiga_vec_est_filter(j,:)';
    omiga_skmatrix = vec2skew(omiga_vec);
    vel = vel_est_filter(j,:)';
    L_r_dot = (-1)*beta_r*L_r-omiga_skmatrix*omiga_skmatrix;
    c_r_dot = (-1)*beta_r*c_r+omiga_skmatrix*vel;
    est_trans_dot = (-1)*Gama_r*(L_r*est_trans-c_r);
    L_r = L_r + L_r_dot;
    c_r = c_r + c_r_dot;
    est_trans = est_trans + est_trans_dot;
    est_trans_filter(j,:) = est_trans;
end