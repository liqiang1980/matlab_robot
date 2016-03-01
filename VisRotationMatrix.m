% this is the code to visualize the rotation sequence of frame.
% author: Qiang Li, Bielefeld

close all;
T1= eye(3);
Ry = roty(-pi/2);
T2 = T1 * Ry;
Rx = rotx(-pi/2);
T3 = T2 * Rx;
for i = 1:4
    T1(i,4) = 0;
    T1(4,i) = 0;
end
T1(4,4) = 1;

for i = 1:4
    T2(i,4) = 0.3;
    T2(4,i) = 0;
end
T2(4,4) = 1;

for i = 1:4
    T3(i,4) = 0.6;
    T3(4,i) = 0;
end
T3(4,4) = 1;

figure;
trplot(T1,'frame','A','color','r','length',0.2);
hold on;
trplot(T2,'frame','B','color','g','length',0.2);
hold on;
trplot(T3,'frame','C','color','b','length',0.2);
hold on;
