function [poly_coef_x, poly_coef_y, ts] = ClosedFormSolver(path,T)
n_order = 7;
n_seg = size(path, 1) - 1;

ts = zeros(n_seg, 1);
dist = zeros(n_seg, 1);
dist_sum = 0;

t_sum = 0;
for i = 1:n_seg
    dist(i) = sqrt((path(i+1, 1) - path(i, 1))^2 + (path(i+1, 2) - path(i, 2))^2);
    dist_sum = dist_sum + dist(i);
end
for i = 1:n_seg-1
    ts(i) = dist(i) / dist_sum * T;
    t_sum = t_sum + ts(i);
end
ts(n_seg) = T - t_sum;
% or you can simply average the time
% for i = 1:n_seg
%     ts(i) = 1;
% end

poly_coef_x = MinimumSnapCloseformSolver(path(:, 1), ts, n_seg, n_order);
poly_coef_y = MinimumSnapCloseformSolver(path(:, 2), ts, n_seg, n_order);
end

function poly_coef = MinimumSnapCloseformSolver(waypoints, ts, n_seg, n_order)
    start_cond = [waypoints(1), 0, 0, 0];
    end_cond =   [waypoints(end), 0, 0, 0];
    %#####################################################
    % you have already finished this function in hw1
    Q = getQ(n_seg, n_order, ts);
    %#####################################################
    % STEP 1: compute M
    M = getM(n_seg, n_order, ts);
    %#####################################################
    % STEP 2: compute Ct
    Ct = getCt(n_seg, n_order);
    C = Ct';
    R = C * inv(M)' * Q * inv(M) * Ct;
    R_cell = mat2cell(R, [n_seg+7 3*(n_seg-1)], [n_seg+7 3*(n_seg-1)]);
    R_pp = R_cell{2, 2};
    R_fp = R_cell{1, 2};
    %#####################################################
    % STEP 3: compute dF
    dF = [waypoints(1),0,0,0]';
    for i=1:n_seg
        dF = [dF;waypoints(i+1)];
    end
    dF = [dF;[0,0,0]'];

    dP = -inv(R_pp) * R_fp' * dF;
    poly_coef = inv(M) * Ct * [dF;dP];
end