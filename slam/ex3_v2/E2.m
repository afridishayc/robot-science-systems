% Input: odo -> 2xT matrix containing odometry readings for T time steps
%        zind -> 1xT vector containing the observed landmark index for
%                T time steps; index is 0 if no landmark observed
%        z -> 1xT cell array containing the (range, bearing) observation
%             for T time steps; z{t} is empty if no observation at time t
%        W -> 2x2 matrix denoting the sensing noise in (range, bearing)
%        x0 -> 3x1 vector denoting the known initial vehicle state
% Output: x_est -> 1xT cell array containing the map state mean
%                  for T time steps (i.e., x_est{t} is a (2M)x1 vector,
%                  where M is the number of landmarks observed by time t)
%         P_est -> 1xT cell array containing the vehicle state covariance
%                  for T time steps (i.e., P_est{t} is a (2M)x(2M) matrix,
%                  where M is the number of landmarks observed by time t)
%         indices -> Mx1 vector containing the landmark index corresponding
%                    to the entries in the state vector, where M is the
%                    number of landmarks observed by the final time step T)
%                    For example, if indices is [15; 4], then the first two
%                    rows of x_est and P_est correspond to landmark 15,
%                    and the next two rows correspond to landmark 4, etc.

function [x_est, P_est, indices] = E2(odo, zind, z, W, x0)
    function x_pred = new_pose(pose_xk, odo_k)
        x_kp1 = pose_xk(1)+odo_k(1)*cos(pose_xk(3));
        y_kp1 = pose_xk(2)+odo_k(1)*sin(pose_xk(3));
        theta_kp1 = pose_xk(3)+odo_k(2);
        x_pred = [x_kp1 y_kp1 theta_kp1]';
    end
    
    function sensor_calc = sensor_cal(current_pose, landmark_loc)
        sensor_calc = [ 
                    sqrt((landmark_loc(2)-current_pose(2))^2 + (landmark_loc(1)-current_pose(1))^2);
                    angdiff(atan2((landmark_loc(2)-current_pose(2)),(landmark_loc(1)-current_pose(1))), current_pose(3)) 
            ];
    end

    function g = g_xz(veh_pos, sensor_obs)
        g = [
                veh_pos(1) + sensor_obs(1) * cos(veh_pos(3) + sensor_obs(2));
                veh_pos(2) + sensor_obs(1) * sin(veh_pos(3) + sensor_obs(2))
            ];
    end 
    
    function jacob_gx = G_x()
        jacob_gx = [ 0 0 0; 0 0 0];
    end
    
    function jacob_gz = G_z(veh_pose, sensor_obs)
        jacob_gz = [ 
                cos(veh_pose(3)+sensor_obs(2)) -sensor_obs(1)*sin(veh_pose(3)+sensor_obs(2));
                sin(veh_pose(3)+sensor_obs(2)) sensor_obs(1)*sin(veh_pose(3)+sensor_obs(2))
            ];
    end
    
    function jacob_hp = H_p(veh_pose, landmark_pos)
        r = norm(veh_pose(1:2)-landmark_pos);
        jacob_hp = [
                    (landmark_pos(1)-veh_pose(1))/r (landmark_pos(2)-veh_pose(2))/r;
                    -(landmark_pos(2)-veh_pose(2))/(r*r) (landmark_pos(1)-veh_pose(1))/(r*r) 
                ];
    end
    
    current_pose = x0;
    current_P = [];
    x_k1 = [];
    p_k1 = [];
    all_indices = [];
    for step_idx = [1:length(z)]
        current_pose = new_pose(current_pose, odo(:,step_idx));
        detected_landmark_index = zind(:, step_idx);
        if detected_landmark_index ~= 0
            disp("landmark detected");
            g_xz_ = g_xz(current_pose, cell2mat(z(:, step_idx)));
            if ~ismember(detected_landmark_index, all_indices)
                % add index
                all_indices = [all_indices;detected_landmark_index];
                % extend x_k1
                x_k1 = [x_k1;g_xz_];
                disp(x_k1)
                % extend Gz
                if length(p_k1) == 0
                    y_z = G_z(current_pose, cell2mat(z(:, step_idx)));
                else
                    p_dim = size(p_k1,1);
%                     y_z = [
%                         eye(p_dim) zeros(p_dim, 2);
%                         G_x() zeros(2, p_dim-3) G_z(current_pose, cell2mat(z(:, step_idx)))
%                     ];
                    y_z = blkdiag(eye(p_dim), G_z(current_pose, cell2mat(z(:, step_idx))));
                end
                p_k1 = y_z * blkdiag(p_k1, W) * y_z';
            else
                % update
                local_landmark_index = find(all_indices==detected_landmark_index);
                h_pi = H_p(current_pose, g_xz_);
                H_x = zeros(2, 2*size(all_indices, 1));
                H_x(1:2,2*local_landmark_index-1:2*local_landmark_index) = h_pi;
                prev_landmark_position = [x_k1(2*local_landmark_index-1); x_k1(2*local_landmark_index)];
                nue = cell2mat(z(:,step_idx)) - sensor_cal(current_pose,prev_landmark_position);
                S = H_x*p_k1*H_x' + eye(2)*W*eye(2)';
                gain = p_k1*H_x'*inv(S);
                x_k1 = x_k1 + gain*nue;
                p_k1 = p_k1 - gain * H_x * p_k1;
            end
        else
            disp("No landmark detected");
        end
        x_est{step_idx} = x_k1;
        P_est{step_idx} = p_k1;
    end
    indices = all_indices;
end