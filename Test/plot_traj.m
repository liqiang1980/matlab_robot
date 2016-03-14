% plot trajectory given velocity.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sponsered by DFG spp-1527: autonmous learning
% author: Qiang Li, Bielefeld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p = zeros(3,1);
p_set = zeros(3,300);
for j = 2:300
    vel = generate_square_shape(j,0.002);
    p_set(:,j) = p_set(:,j-1)+vel;
end
plot(p_set(1,:),p_set(2,:))