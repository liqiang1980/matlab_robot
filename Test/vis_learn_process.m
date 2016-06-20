function vis_learn_process(est, omiga_vec_est, vel_real_est, vel_est,real_link,len)
disp('est link value');
est_link = est(end,:)
disp('real link value');
real_link
figure(4);
subplot(3,1,1);
plot(est(160:len,1));
subplot(3,1,2);
plot(est(160:len,2));
subplot(3,1,3);
plot(est(160:len,3));
title('estimation convergency curve');

A = omiga_vec_est;
r1 = est_link;  
B1 = repmat(r1,len,1);
C1 = -1*cross(A,B1);
        
r2 = real_link';
B2 = repmat(r2, len,1);
C2 = -1*cross(A,B2);
start_ind = 2;
figure(7);plot(C1(start_ind:len,1));hold on;plot(C2(start_ind:len,1));
hold on;plot(vel_real_est(start_ind:len,1));
hold on;plot(vel_est(start_ind:len,1));

figure(8);plot(C1(start_ind:len,2));hold on;plot(C2(start_ind:len,2));
hold on;plot(vel_real_est(start_ind:len,2));
hold on;plot(vel_est(start_ind:len,2));


figure(9);plot(C1(start_ind:len,3));hold on;plot(C2(start_ind:len,3));
hold on;plot(vel_real_est(start_ind:len,3));
hold on;plot(vel_est(start_ind:len,3));

end