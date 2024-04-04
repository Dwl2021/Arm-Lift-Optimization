function Q = getQ(n_seg, n_order, T)
    Q = [];
    
    for k = 1:n_seg
        %#####################################################
        Q_k=zeros(n_order+1,n_order+1);
        for i=0:n_order
            for l=0:n_order
%                 Q_k(i+1,l+1)=fac(i)*fac(l)/(i+l-7)*T(k)^(i+l-7);
                tmp =  (i + l - n_order) * T(k)^(i + l - n_order);
                if tmp == 0
                    Q_k(i+1,l+1) = 0;
                else
                    Q_k(i+1,l+1) = continue_mul(i, n_order) * continue_mul(l, n_order) / tmp;
                end
            end
        end
        Q = blkdiag(Q, Q_k);
    end
end

function [res] = continue_mul(x, n_order)
    res = 1;
    for i = 1:(n_order+1)/2
        res = res * (x + i - 1);
    end
end