close all
load nvest.txt
figure(1)
subplot(3,1,1);
plot(nvest(:,1));
subplot(3,1,2);
plot(nvest(:,2));
subplot(3,1,3);
plot(nvest(:,3));

figure(2);
subplot(2,1,1);
plot(nvest(60:end,4));
subplot(2,1,2);
plot(nvest(60:end,5));

figure(3)
subplot(3,1,1);
plot(nvest(:,6));
subplot(3,1,2);
plot(nvest(:,7));
subplot(3,1,3);
plot(nvest(:,6),nvest(:,7));

figure(4);
subplot(3,1,1);
plot(nvest(:,8));
subplot(3,1,2);
plot(nvest(:,9));
subplot(3,1,3);
plot(nvest(:,10));

figure(5);
subplot(3,1,1);
plot(nvest(:,11));
subplot(3,1,2);
plot(nvest(:,12));
subplot(3,1,3);
plot(nvest(:,13));