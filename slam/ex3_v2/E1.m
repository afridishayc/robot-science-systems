% Input: odo -> 2xT matrix containing odometry readings for T time steps
%        zind -> 1xT vector containing the observed landmark index for
%                T time steps; index is 0 if no landmark observed
%        z -> 1xT cell array containing the (range, bearing) observation
%             for T time steps; z{t} is empty if no observation at time t
%        V -> 2x2 matrix denoting the process noise in (forward, angular)
%        W -> 2x2 matrix denoting the sensing noise in (range, bearing)
%        x0 -> 3x1 vector denoting the initial vehicle state mean
%        P0 -> 3x3 matrix denoting the initial vehicle state covariance
%        map -> Robotics toolbox Map object containing the known map
%               (known landmarks) for localization
% Output: x_est -> 1xT cell array containing the vehicle state mean
%                  for T time steps (i.e., x_est{t} is a 3x1 vector)
%         P_est -> 1xT cell array containing the vehicle state covariance
%                  for T time steps (i.e., P_est{t} is a 3x3 matrix)

function [x_est, P_est] = E1(odo, zind, z, V, W, x0, P0, map)
    % new pose
    function x_pred = new_pose(pose_xk, odo_k)
        x_kp1 = pose_xk(1)+odo_k(1)*cos(pose_xk(3));
        y_kp1 = pose_xk(2)+odo_k(1)*sin(pose_xk(3));
        theta_kp1 = pose_xk(3)+odo_k(2);
        x_pred = [x_kp1 y_kp1 theta_kp1]';
    end
    
    function F_x_jacob = F_x(v_state, odo_k)
        delta_d = odo_k(1);
        theta_v  = v_state(3);
        F_x_jacob = [1 0 -delta_d*sin(theta_v); 0 1 delta_d*cos(theta_v); 0 0 1];
    end

    function F_v_jacob = F_v(v_state)
        theta_v = v_state(3);
        F_v_jacob = [cos(theta_v) 0; sin(theta_v) 0; 0 1];
    end

    function H_x_jacob = H_x(v_state, landmark_positon)
        r = norm(v_state(1:2)-landmark_positon);
        H_x_jacob = [
                        -(landmark_positon(1)-v_state(1))/r -(landmark_positon(2)-v_state(2))/r 0;
                        (landmark_positon(2)-v_state(2))/(r*r) -(landmark_positon(1)-v_state(1))/(r*r) -1
                    ];
    end

    function H_w_jacob = H_w()
        H_w_jacob = [1 0; 0 1];
    end
    function sensor_calc = sensor_cal(current_pose, landmark_loc)
        sensor_calc = [ 
                    sqrt((landmark_loc(2)-current_pose(2))^2 + (landmark_loc(1)-current_pose(1))^2);
                    angdiff(atan2((landmark_loc(2)-current_pose(2)),(landmark_loc(1)-current_pose(1))), current_pose(3)) 
            ];
    end

    current_pose = x0;
    current_P = P0;
    for step_idx = [1:length(z)]
        x_pred_k1 = new_pose(current_pose, odo(:,step_idx));   
        Fx_mat = F_x(x_pred_k1, odo(:,step_idx));
        Fv_mat = F_v(x_pred_k1);
        P_pred = Fx_mat * current_P * Fx_mat' + Fv_mat * V * Fv_mat';
        if zind(step_idx) ~= 0
            % update
            disp(cell2mat(z(:,step_idx)));
            nue = cell2mat(z(:,step_idx)) - sensor_cal(x_pred_k1, map.landmark(zind(step_idx)));
            Hx = H_x(x_pred_k1, map.landmark(zind(step_idx)));
            Hw = H_w();
            K = P_pred * Hx' * inv(Hx*P_pred*Hx'+Hw*W*Hw');
            current_pose = x_pred_k1 + K*nue;
            current_P = P_pred - K*Hx*P_pred;
        else
            current_pose = x_pred_k1;
            current_P = P_pred;
        end
        x_est{step_idx} = current_pose;
        P_est{step_idx} = current_P;
    end
end