
function v3 = V3()
    function [true_normal, fit_normal] = planeNormals(T, P)
       x0 = mean(P');
       P = bsxfun(@minus, P, x0');
       J = P*P';
       [x,lambda] = eig(J);
       [min_lambda, lambda_ind] = min(diag(lambda));
       fit_normal = x(:,lambda_ind)';
       true_normal = T.SO3.a';
    end
    
    function d = distance_to_plane(points, centroid, normal)
        d = [];
        for point = points
            d = [d abs(dot(normal, centroid-point') / norm(normal))];
        end
    end
    
    
    function fit_normal = weightedPlaneNormals(T, P)
        weights = ones(1, size(P, 2));
        x0 = mean(P');
        P_bar = bsxfun(@minus, P, x0');
        for i = 1:100
            J = weights.*P_bar*P_bar';
            [x,lambda] = eig(J);
            [min_lambda, min_lambda_ind] = min(diag(lambda));
            fit_normal = x(:,min_lambda_ind)';
            distances = distance_to_plane(P, x0, fit_normal);
            hw = mean(distances);
            weights = hw^2 *  ((hw^2 + (distances.^2)).^-1);
        end
    end

    T = SE3(1,2,3) * SE3.rpy(0.3, 0.4, 0.5);
    P = mkgrid(10, 1, 'pose', T);
    P = P + 0.02*randn(size(P));
    
    % normal of the fit plane is very close to the real plane's normal
    [true_normal, fit_normal] =  planeNormals(T, P)
    
    % replacing some points with outliers 
    P_noisy = mkgrid(10, 1, 'pose', T);
    P_noisy = P_noisy + 0.02*randn(size(P));
    P_noisy(:, 95:100) = 20*randn(3,6)
    
    %normals are very different from each other
    [true_normal, fit_normal] =  planeNormals(T, P_noisy)
    
    % weighted normal; RESULTS ARE STILL VERY OFF FROM THE FIT NORMAL
    fit_normal= weightedPlaneNormals(T, P_noisy)
end


