L1 = Link('d', 0.5, 'alpha', -pi/2, 'a', 0);
L2 = Link('d', 0.15, 'alpha', pi/2, 'a', 0);
L3 = Link('theta', -pi/2, 'alpha', 0, 'a', 0);
% L3.sigma = 1;
% L3.qlim = [0.1 1]; 
myRRP = SerialLink([L1 L2 L3], 'name', 'RRP');
myRRP.plot([0.0 0.0 0.2],'workspace', [-3 3 -3 3 -3 3]);
myRRP.teach