% load 'nv_hat_sim_1.mat'
% nv_hat_set = nv_hat_sim_1;

x_desire =0.9417* ones(1,80);
y_desire =-0.0072* ones(1,80);
z_desire =0.3364* ones(1,80);
figure(2);
title('tool normal direction')
subplot(3,1,1);
plot(n_hat_set(1,:),'b');
hold on;
plot(x_desire,'r');
ylabel('x');
subplot(3,1,2);
plot(n_hat_set(2,:),'b');
hold on;
plot(y_desire,'r');
ylabel('y');
subplot(3,1,3);
plot(n_hat_set(3,:),'b');
hold on;
plot(z_desire,'r');
ylabel('z');
xlabel('iterative(sliding) times');