square = [-2, 2, 2, -2;-2, -2, 2, 2];
figure;
xmin = -5;
xmax = 25;
ymin = -5;
ymax = 125;
axis equal
axis([xmin xmax ymin ymax])
hold on
N = 50;
t = linspace(0,20,N); 
trajectory = 1/4*t.^2;
scale = linspace(1,0.25,N);
for i = 1:N
    % object moving along projectile
    h = fill(square(1,:) + t(i),square(2,:) + trajectory(i),'r');
    hold on;
    % scales the figure from 100% to 25%
    fill(scale(i)*square(1,:) + t(i), scale(i)*square(2,:) + trajectory(i), 'r');
    hold on;
    pause(0.3)
end