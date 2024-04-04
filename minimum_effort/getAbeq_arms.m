function [Aeq, beq]= getAbeq_arms(n_seg, n_order, ts, start_cond, end_cond)
    n_all_poly = n_seg * (n_order+1);
    state_num = (n_order+1) / 2;    
 
    % state in start
    Aeq_start = zeros(state_num, n_all_poly);
    beq_start =  start_cond;
    Aeq_start(1 : state_num, 1 : n_order+1) = getCoeff_arms(0, n_order); 

    % state in end
    Aeq_end = zeros(state_num, n_all_poly);
    beq_end =  end_cond;
    t = ts(length(ts));
    Aeq_end(1 : state_num, n_all_poly - n_order : n_all_poly) = getCoeff_arms(t, n_order);

    % continuity constrain between each 2 segments
    Aeq_con = [];
    beq_con = [];
    for i = 1:state_num
        Aeq_con = [Aeq_con;zeros(n_seg-1, n_all_poly)];
        beq_con = [beq_con;zeros(n_seg-1, 1)];
    end
    n = state_num;
    for k = 0:1:n_seg-3 % here k is the index of segments
            Aeq_con(1+n*k:n+n*k,1+2*n*k:2*n+2*n*k) = getCoeff_arms(ts(k+1), n_order);
            Aeq_con(1+n*k:n+n*k,1+2*n*(k+1):2*n+2*n*(k+1)) = -getCoeff_arms(0, n_order);            
    end
    
    % combine all components to form Aeq and beq:  
    Aeq = [Aeq_start; Aeq_end; Aeq_con];
    beq = [beq_start; beq_end; beq_con];
end

