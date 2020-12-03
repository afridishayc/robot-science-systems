% input: f -> a 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        posGoal -> 3x1 vector denoting the target position to move to
% output: q -> 1x9 vector of joint angles that cause the end
%              effector position to reach <position>
%              (orientation is to be ignored)

function q = Q2(f, qInit, posGoal)
    step_size = 0.1;
    proximity = 0.05;
    qCurrent = qInit;
    tm = f.fkine(qCurrent);
    current_pose = tm.t;
    diff = posGoal - current_pose;
    current_distance = norm(diff)
    while current_distance > proximity
        for i = 1:10
            tm = f.fkine(qCurrent);
            current_pose = tm.t;
            diff = posGoal - current_pose;
            current_distance = norm(diff)
            diff = [diff; [0; 0; 0]];
            jacob = f.jacob0(qCurrent);
            inv_jacob = pinv(jacob);
            dq = inv_jacob * diff;
            qCurrent = qCurrent + (step_size * dq')
        end
    end
    q = qCurrent
end