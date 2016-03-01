% table robot kinematics model, DH representation
%author, QiangLi, CITEC Bielefeld University
%time: 14,May,2015

function r = mdl_tablerobot()
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
    r = tablerobot;
end
