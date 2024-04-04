function [poly_coef_x, poly_coef_y, ts] = QPSolver(path,T, max_vel ,max_acc, n_order)

n_seg = size(path, 1) - 1;

ts = zeros(n_seg, 1);
dist = zeros(n_seg, 1);
dist_sum = 0;

% t_sum = 0;
% for i = 1:n_seg
%     dist(i) = sqrt((path(i+1, 1) - path(i, 1))^2 + (path(i+1, 2) - path(i, 2))^2);
%     dist_sum = dist_sum + dist(i);
% end
% for i = 1:n_seg-1
%     ts(i) = dist(i) / dist_sum * T;
%     t_sum = t_sum + ts(i);
% end
% ts(n_seg) = T - t_sum;
% or you can simply average the time
for i = 1:n_seg
    ts(i) = 1;
end

poly_coef_x = MinimumSnapQPSolver(path(:, 1), ts, n_seg, n_order,max_vel,max_acc);
poly_coef_y = MinimumSnapQPSolver(path(:, 2), ts, n_seg, n_order,max_vel,max_acc);
end


function poly_coef = MinimumSnapQPSolver(waypoints, ts, n_seg, n_order,max_vel,max_acc)
    start_cond = [waypoints(1), 0, 0, 0];
    end_cond   = [waypoints(end), 0, 0, 0];
    %#####################################################
    % STEP 1: compute Q of p'Qp
    Q = getQ(n_seg, n_order, ts);
    %#####################################################
    % STEP 2: compute Aeq and beq 
    [Aeq, beq] = getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond);
    [Auneq, buneq] = getAbuneq(n_seg, n_order,ts, max_vel, max_acc);
    f = zeros(size(Q,1),1);
    poly_coef = quadprog(Q,f,Auneq,buneq,Aeq, beq);
end