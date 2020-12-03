T = SE3(1,2,3) * SE3.rpy(0.3, 0.4, 0.5);
P = mkgrid(10, 1, 'pose', T);

p1 = cam1.plot(P, 'o');
p2 = cam2.plot(P, 'o');