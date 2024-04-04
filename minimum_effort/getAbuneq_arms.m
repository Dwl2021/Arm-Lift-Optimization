function [Auneq, buneq]= getAbuneq_arms(n_seg, n_order, ts, max_vel, max_acc)
    n_all_poly = n_seg*(n_order+1);
   
    % velocity maximum constrain in all middle waypoints
    Auneq_wp_v = zeros(n_seg-1, n_all_poly);
    buneq_wp_v = zeros(n_seg-1, 1);
    for k = 0:1:n_seg-2 % here k is the index of segments
        buneq_wp_v(k+1, 1) = max_vel;
        coeff = getCoeff_arms(ts(k+1),n_order);
        Auneq_wp_v(k+1, 1+k*8:8+k*8) = coeff(2, :);  
    end
    
    Auneq_wp_a = zeros(n_seg-1, n_all_poly);
    buneq_wp_a = zeros(n_seg-1, 1);
    for k = 0:1:n_seg-2
        buneq_wp_a(k+1, 1) = max_acc;
        coeff = getCoeff_arms(ts(k+1),n_order);
        Auneq_wp_a(k+1, 1+k*8:8+k*8) = coeff(3, :);
    end

    % combine all components to form Auneq and buneq:  
    Auneq = [Auneq_wp_v;-Auneq_wp_v;Auneq_wp_a;-Auneq_wp_a];
    buneq = [buneq_wp_v;buneq_wp_v;buneq_wp_a;buneq_wp_a];
end
