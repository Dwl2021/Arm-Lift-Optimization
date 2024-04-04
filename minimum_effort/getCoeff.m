function coeff = getCoeff(t, n_order)
    % 初始化系数矩阵
    coeff = zeros((n_order+1)/2-1, n_order+1);
    for k = 0:(n_order+1)/2-1  % 对于每一行（即每阶导数）
        for i = k:n_order  % 只有当列索引大于等于当前导数阶数时，系数才非0
            coeff(k+1, i+1) = factorial(i) / factorial(i-k) * t^(i-k);
        end
    end
end
