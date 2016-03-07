% Jacobian and inverse Jacobian function test in the RTB
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sponsered by DFG spp-1527: autonmous learning
% author: Qiang Li, Bielefeld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
kuka_robot = loadrobot('kukalwr');
q = randn(1,7);

T_old = eye(4);
for i = 1:1:100
    %compute the current homogeneous matrix
    T = kuka_robot.fkine(q);
    delta_p(:,i) = T(1:3,4)-T_old(1:3,4);
    %Jacobian and inverse
    J = kuka_robot.jacob0(q);
    lamda = 0.1;
    Ji = J'*inv(J*J'+lamda^2*eye(6));
    %deta_v in global
    delta_v = [0.01,0,0,0,0,0];
    q_dot = Ji*delta_v';
    q = q + q_dot';
    T_old = T;
end