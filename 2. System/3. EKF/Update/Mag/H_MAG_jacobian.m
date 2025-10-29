%==========================================================================
% Niel Theron
% 02-10-2025
%==========================================================================
% Numerical Jacobian of the magnetometer measurement function H_MAG_function
% with respect to the state vector x_est using central differences.
%==========================================================================
function H = H_MAG_jacobian(x_est, we, t)

% Parameters ==============================================================
eps = 1e-6;   % Perturbation step
n   = length(x_est);                       % State vector size
m   = length(H_MAG_function(x_est, we, t));% Measurement vector size (3)

% Allocate Jacobian
H = zeros(m, n);

% Loop over each state variable ===========================================
for i = 1:n
    dx = zeros(n, 1);
    dx(i) = eps;

    % Perturbed states
    z_p = H_MAG_function(x_est + dx, we, t);
    z_m = H_MAG_function(x_est - dx, we, t);

    % Central difference derivative
    H(:, i) = (z_p - z_m) / (2 * eps);
end

end
