function kuka_robot = loadrobot(name)
switch name
  case char('kukalwr')
      %set base transform
      T =[0.433466   -0.250194    0.865743      0.0823;
        0.499272    0.866445 0.000418414       0.897;
        -0.750223    0.432059    0.500489      0.2975;
        0           0           0           1];
      %configure the robot kinematics chain
      d1 = 0.31;
      d2 = 0.4;
      d3 = 0.39;
      d4 = 0.078;
      alpha1 = pi/2;
      alpha2 = -pi/2;
      alpha3 = -pi/2;
      alpha4 = pi/2;
      alpha5 = pi/2;
      alpha6 = -pi/2;
          
      % theta, d, a, alpha sigma(0:revolute)in sequence
      L(1) = Link([ 0 d1 0 alpha1 0], 'standard');
      L(1).qlim = [-pi*165/180,pi*165/180];
      L(2) = Link([ 0 0 0 alpha2 0], 'standard');
      L(2).qlim = [-pi*110/180,pi*110/180];
      L(3) = Link([0 d2 0 alpha3 0],'standard');
      L(3).qlim = [-pi*165/180,pi*165/180];
      L(4) = Link([0 0 0 alpha4 0],'standard');
      L(4).qlim = [-pi*110/180,pi*110/180];
      L(5) = Link([0 d3 0 alpha5 0],'standard');
      L(5).qlim = [-pi*165/180,pi*165/180];
      L(6) = Link([0 0 0 alpha6 0],'standard');
      L(6).qlim = [-pi*110/180,pi*110/180];
      L(7) = Link([0 d4 0 0 0],'standard');
      L(7).qlim = [-pi*165/180,pi*165/180];
      kuka_robot = SerialLink(L, 'name', 'kuka_lwr','base',T);
      fprintf('load kuka lwr mode successfully');
end

end