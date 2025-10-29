function zhat_i = H_ST_function_i(x_est, star_idx)
    zhat_all = H_ST_function(x_est);   % 3 x d
    zhat_i   = zhat_all(:, star_idx);  % 3x1
end
