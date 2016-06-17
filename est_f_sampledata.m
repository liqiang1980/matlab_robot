close all;
load est.txt;
end_num = 15000;
start_num = 15;
figure(1);
subplot(3,1,1);
plot(est(start_num:end_num,1));
subplot(3,1,2);
plot(est(start_num:end_num,2));
subplot(3,1,3);
plot(est(start_num:end_num,3));

figure(2);
subplot(3,1,1);
plot(est(start_num:end_num,4));
subplot(3,1,2);
plot(est(start_num:end_num,5));
subplot(3,1,3);
plot(est(start_num:end_num,6));


figure(3);
subplot(3,1,1);
plot(est(start_num:end_num,7));
subplot(3,1,2);
plot(est(start_num:end_num,8));
subplot(3,1,3);
plot(est(start_num:end_num,9));

disp('meanx')
mean(est(6000:end_num,7))
disp('meany')
mean(est(6000:end_num,8))
disp('meanz')
mean(est(6000:end_num,9))

figure(4);
subplot(2,1,1);
plot(est(start_num:end_num,10));
ylim([-0.03,0.03]);
subplot(2,1,2);
plot(est(start_num:end_num,11));
ylim([-0.03,0.03]);

figure(5);
subplot(2,1,1);
plot(est(start_num:end_num,13));
subplot(2,1,2);
plot(est(start_num:end_num,14));

figure(6)
subplot(2,1,1);
plot(est(start_num:end_num,10));
subplot(2,1,2);
plot(est(start_num:end_num,13));

A = est(start_num:end_num,4:6)';
% r = [0.0427471,-0.14285,0.1604];
% r = [0.037,-0.077,0.126];
% r1 = [mean(est(start_num:end_num,7)),mean(est(start_num:end_num,8)),mean(est(start_num:end_num,9))];
r1 = [0,0.13,0.22];
B1 = repmat(r1',1,end_num-14);
C1 = -1*cross(A,B1);


r2 = [-0.007,0.0614,0.1982];
B2 = repmat(r2', 1,end_num-14);
C2 = -1*cross(A,B2);

figure(7);plot(C1(1,100:end_num-14));hold on;plot(C2(1,100:end_num-14));
hold on;plot(est(start_num+100:end_num,15));ylim([-0.02,0.02]);
figure(8);plot(C1(2,100:end_num-14));hold on;plot(C2(2,100:end_num-14));
hold on; plot(est(start_num+100:end_num,16));ylim([-0.02,0.02]);
figure(9);plot(C1(3,100:end_num-14));hold on;plot(C2(3,100:end_num-14));
hold on; plot(est(start_num+100:end_num,17));ylim([-0.02,0.02]);