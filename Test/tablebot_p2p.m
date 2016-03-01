%tablebot move from one point to another point.
%author, QiangLi, CITEC Bielefeld University
%time: 14,May,2015

%load robot kinematics
tablerobot = mdl_tablerobot();

startP = [1 0 -0.05];
endP = [0.503 0.330 0.110];

T0= eye(4);
T0(1,4) = startP(1);
T0(2,4) = startP(2);
T0(3,4) = startP(3);
T1 = eye(4);
T1(1,4) = endP(1);
T1(2,4) = endP(2);
T1(3,4) = endP(3);
nFrames = 200;
mov(1:nFrames) = struct('cdata',[], 'colormap',[]);
tc = ctraj(T0, T1, nFrames);
m = [1 1 1 0 0 0];

trpoints = size(nFrames,3);
for i = 1:1:nFrames
    trpoints(i,1:3) = tc(1:3,4,i)';
end

Q = zeros(1,3);
for i = 1:1:nFrames
    Q = tablerobot.ikine(tc(:,:,i),Q,m);
    plotp(startP','g*');
    hold on;
    plotp(endP','rs');
    hold on;
    plotp(trpoints(i,:)','b*');
    hold on;
    tablerobot.plot(Q,'workspace', [-1 1 -1 1 -1 1]);
    mov(i) = getframe(gca);
end
close(gcf);
movie2avi(mov, 'Tablebot.avi', 'compression','None', 'fps',10);