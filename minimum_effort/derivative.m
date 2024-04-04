% 给定多项式系数求导数
function [d] = derivative(c, begin, endd, step ,order)
    d = [];
    for i=begin:step:endd
        coeff = getCoeff(i,length(c)-1);
        p = coeff(order+1,:);
        d = [d, p*c];
    end
end