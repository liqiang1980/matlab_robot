function vis_learn_process(est, omiga_vec_est, vel_real_est, vel_est,real_link,len)
disp('est link value');
est_link = est(end,:)
disp('real link value');
real_link
mat_real_link = repmat(real_link,1,len) ;
figure(4);
title('estimation convergency curve');
subplot(3,1,1);
plot(est(1:len,1)-mat_real_link(1,1:len)');
ylabel('x');
grid on;
subplot(3,1,2);
plot(est(1:len,2)-mat_real_link(2,1:len)');
ylabel('y');grid on;
subplot(3,1,3);
plot(est(1:len,3)-mat_real_link(3,1:len)');
xlabel('iterative(rotate) times');
ylabel('z');
grid on;

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