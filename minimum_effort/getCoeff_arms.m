function coeff = getCoeff_arms(t, n_order)
    % 初始化系数矩阵
    coeff = zeros((n_order+1)/2-1, n_order+1);
    for k = 0:(n_order+1)/2-1 
        for i = k:n_order 
            coeff(k+1, i+1) = factorial(i) / factorial(i-k) * t^(i-k);
        end
    end
end
