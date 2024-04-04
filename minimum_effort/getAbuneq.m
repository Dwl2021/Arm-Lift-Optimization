function [Auneq, buneq] = getAbuneq(n_seg, n_order, ts, y)
    n_all_poly = n_seg*(n_order+1);
    Auneq = zeros(n_seg-1, n_all_poly);
    buneq = zeros(n_seg-1, 1);
    for k = 1:n_seg-1 % here k is the index of segments
        buneq(k, 1) = y(k+1);
        coeff = getCoeff(ts(k), n_order);
        Auneq(k, 1+(k-1)*(n_order+1):(n_order+1)+(k-1)*(n_order+1)) = coeff(2, :); 
    end
end
