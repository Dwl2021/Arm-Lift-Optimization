clear;close all;clc;
n_order = 3;

y = [8, 8, 6, 9, 5, 7, 8, 9, 6, 10];
x = 1:size(y, 2);
n_seg = size(y, 2) - 1;

% plot the data
figure;
axis([0, size(y, 2)+2, 0, 20]);
hold on;
plot(x, y, '-o');
hold on;

start_state = y(1);
end_state = y(end);


for i = 1:n_seg
    ts(i) = 1;
end

coff = MinimumSnapQPSolver(start_state, end_state, y, ts, n_seg, n_order);

X_n = [];
Y_n = [];
k = 1;
tstep = 0.01;
for i=0:n_seg-1
    P = coff((n_order+1)*i+1:(n_order+1)*i+(n_order+1));
    for t=0:tstep:ts(i+1)
        Y_n(k) = polyval(flip(P),t);
        X_n(k) = 1 + (k-1)/100;
        k = k+1;
    end
end
plot(X_n, Y_n ,'Color','#FFA500','LineWidth',2);
hold on;


function poly_coef = MinimumSnapQPSolver(start_state, end_state, y, ts, n_seg, n_order)
    start_cond = zeros((n_order+1)/2, 1);
    start_cond(1) = start_state;
    end_cond = zeros((n_order+1)/2, 1);
    end_cond(1) = end_state;
    % get the weight of objective function
    Q = getQ_p(n_seg, n_order, ts);
    % get the linear part of the objective function
    [Aeq, beq] = getAbeq(n_seg, n_order, ts, start_cond, end_cond);
    f = zeros(size(Q,1),1);
    ub = inf(n_seg*(n_order+1),1)*10;
    lb = -inf(n_seg*(n_order+1),1)*3;
    for i = 2:n_seg
        ub(1 + (i-1) * (n_order+1),1) = y(i);
        lb(1 + (i-1) * (n_order+1),1) = 3;
    end
%     for i = 2:n_seg
%         f(1+(i-1)*(n_order+1)) = -2*(y(i));
%     end
    poly_coef = quadprog(Q,f,[],[],Aeq, beq,lb,ub);
%     poly_coef = quadprog(Q,f,[],[],Aeq, beq);
end