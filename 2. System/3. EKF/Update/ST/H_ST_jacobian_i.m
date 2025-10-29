function H_i = H_ST_jacobian_i(x_est, star_idx)
    H_all = H_ST_jacobian(x_est);      % 3 x d x n
    H_i   = squeeze(H_all(:, star_idx, :));  % 3 x n
end