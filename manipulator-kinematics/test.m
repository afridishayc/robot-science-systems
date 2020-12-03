L(1) = Link([0 0 0 pi/2]);
L(2) = Link([0 0 0 -pi/2]);
L(3) = Link([0 0.4318 0 -pi/2]);
L(4) = Link([0 0 0 1.571]);
L(5) = Link([0 0.4318 0 pi/2]);
L(6) = Link([0 0 0 -pi/2]);
L(7) = Link([0 0 0 0]);
L(8) = Link([0 0 0.2 0]);
L(9) = Link([0 0 0.2 0]);

f1 = SerialLink(L, 'name', 'f1')


qInit = [0 -pi/4 0 pi/2 0 pi/2 0 -1 1];
goalPose = [0.65; 0.0; -0.75]
step_size = 0.05
nTheta = 15;
   radius = 0.3;
        theta = linspace(0, 2*pi, nTheta);
        circle = repmat(spherePos, 1, nTheta) + radius * [cos(theta); sin(theta); zeros(1, nTheta)];

        % Get initial arm pose at the start of the circle
        qInit = Q1(f1, circle(:,1));
        disp(qInit )