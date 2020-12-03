


function act = act()
    robot = create_robot()
    q = [0 -pi/4 0 -pi/4];
    x1 = [0 0 0]'
    T2 = robot.A(1,q) * robot.A(2,q) * robot.A(3,q);
    x2 = T2.t
    T3 = T2 * robot.A(4,q);
    x3 = T3.t
    
    ticks = linspace(0, 1, 11);
    n = 11;
    x12 = repmat(x1, 1, n) + repmat(x2 - x1, 1, n) .* repmat(ticks, 3, 1)
    
end

function robot = create_robot()
    L(1) = Link([0 0 0 1.571]);
    L(2) = Link([0 0 0 -1.571]);
    L(3) = Link([0 0.4318 0 -1.571]);
    L(4) = Link([0 0 0.4318 1.571]);    
    robot = SerialLink(L, 'name', 'robot');
end