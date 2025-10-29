%==========================================================================
% Niel Theron
% 02-10-2025
%==========================================================================
% Numerical Jacobian of the gyroscope measurement function H_GYR_function
% with respect to the state vector x_est using central differences.
%
% Usage:
%   H = H_GYR_jacobian(x_est, Mu)
%
% The function returns a 3-by-n Jacobian matrix where n = length(x_est).
%==========================================================================
function H = H_GYR_jacobian(x_est, Mu)

% Parameters ==============================================================
eps = 1e-6; % fixed perturbation step

n = length(x_est);

% Evaluate measurement size (should be 3)
z0 = H_GYR_function(x_est, Mu);
m  = length(z0);

% Pre-allocate Jacobian
H = zeros(m, n);

% Central-difference loop
for i = 1:n
    dx = zeros(n,1);
    dx(i) = eps;

    z_p = H_GYR_function(x_est + dx, Mu);
    z_m = H_GYR_function(x_est - dx, Mu);

    H(:,i) = (z_p - z_m) / (2*eps);
end

end