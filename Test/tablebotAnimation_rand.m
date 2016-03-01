%tablebot motion animation, q1 will continuous move and q2 is randomly
%generated. q3 is continuous moving
%author, QiangLi, CITEC Bielefeld University
%time: 14,May,2015
tablerobot = mdl_tablerobot();
for q1 = 0:0.5:6.2
    for prislen = 0.0:0.2:1
        q2 = 3.14 * rand(1);
        qz = [q1,q2,prislen];
        tablerobot.plot(qz,'workspace', [-1 1 -1 1 -1 1]);
    end
end