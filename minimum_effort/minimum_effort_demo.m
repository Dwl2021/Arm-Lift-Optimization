clear;close all;clc;
n_order = 7;

y = [10, 10, 10, 10, 10, 10, 10, 10, 9, 10, 10, 10, 10, 10, 10, 3, 4, 6, 6];

x = 1:size(y, 2);
n_seg = size(y, 2) - 1;

% plot the data
figure;
axis([0, size(y, 2)+2, 0, 20]);
hold on;
plot(x, y, '-o');
hold on;

% set the start state and end state
start_state = y(1);
end_state = y(end);

% define the segment time
for i = 1:n_seg
    ts(i) = 1;
end
ts(end) = 1;

% solve the QP program
coff = MinimumSnapQPSolver(start_state, end_state, y, ts, n_seg, n_order);


% plot the poly
X_n = [];
Y_n = [];

k = 1;
tstep = 0.01;
for i=0:n_seg-1
    P = coff((n_order+1)*i+1:(n_order+1)*i+(n_order+1));
    for t=0:tstep:ts(i+1)
        X_n(k) = 1 + (k-1)/100;
        Y_n(k) = polyval(flip(P),t);
        k = k+1;
    end
end
plot(X_n, Y_n ,'Color','#FFA500','LineWidth',2);

% plot the derivative
% d  = [];
% for i=0:n_seg-1
    % P = coff((n_order+1)*i+1:(n_order+1)*i+(n_order+1));
    % d = [d, derivative(P, 0, ts(i+1),tstep, 1)];
% end
% plot(X_n,d)
% hold on;


function poly_coef = MinimumSnapQPSolver(start_state, end_state, y, ts, n_seg, n_order)
    start_cond = zeros((n_order+1)/2, 1);
    start_cond(1) = start_state;
    end_cond = zeros((n_order+1)/2, 1);
    end_cond(1) = end_state;
    % get the weight of objective function
    Q = getQ_p_arms(n_seg, n_order, ts);
    % get the linear part of the objective function
    [Aeq, beq] = getAbeq_arms(n_seg, n_order, ts, start_cond, end_cond);
%     [Auneq, buneq]= getAbuneq(n_seg, n_order, ts, 1.0,1.0);
    f = zeros(size(Q,1),1);
    ub = inf(n_seg*(n_order+1),1)*10;
    lb = -inf(n_seg*(n_order+1),1)*3;
    for i = 2:n_seg
        ub(1 + (i-1) * (n_order+1),1) = y(i)-0.5;
        ub(2 + (i-1) * (n_order+1),1) = 5;
        lb(1 + (i-1) * (n_order+1),1) = 1;
        lb(2 + (i-1) * (n_order+1),1) = -5;
    end
    poly_coef = quadprog(Q,f,[], [],Aeq, beq,lb,ub);
end