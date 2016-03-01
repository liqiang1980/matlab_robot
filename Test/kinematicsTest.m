% robot kinematics model test
%author, QiangLi, CITEC Bielefeld University
%time: 14,May,2015

l1 = -0.05;
l2 = 0.4;
l3 = -0.2;
l4 = 0.6;
% theta, d, a, alpha in sequence
L(1) = Link([ 0 l1+l3 l2 0 0], 'standard');
L(1).qlim = [-pi,pi];
L(2) = Link([ 0 0 l4 0 0], 'standard');
L(2).qlim = [-pi,pi];
L(3) = Link([0 0 0 0 1],'standard');
L(3).qlim = [-1,1];
tablerobot = SerialLink(L, 'name', 'RRP');
Q = [0,0,0.2];
tablerobot.plot(Q,'workspace', [-1 1 -1 1 -1 1]);

T0= eye(4);
T0(1,4) = 1;
T0(2,4) = 0;
T0(3,4) = -0.05;
T1 = eye(4);
T1(1,4) = 0.903;
T1(2,4) = 0.330;
T1(3,4) = 0.110;

tc = ctraj(T0, T1, 100);
m = [1 1 1 0 0 0];
q0 = [0.3,0.2,0];
for i = 1:1:100
Q = tablerobot.ikine(tc(:,:,i),q0,m);
tablerobot.plot(Q,'workspace', [-1 1 -1 1 -1 1]);
end
