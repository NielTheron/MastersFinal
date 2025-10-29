%==========================================================================
% Niel Theron
% 02-10-2025
%==========================================================================
% Numerical Jacobian of the coarse sun sensor measurement function
% H_CSS_function with respect to the state vector x_est using central
% differences.
%
% Usage:
%   H = H_CSS_jacobian(x_est)
%
% The function returns a 3-by-n Jacobian matrix where n = length(x_est).
%==========================================================================
function H = H_CSS_jacobian(x_est)

% Parameters ==============================================================
eps = 1e-6; % fixed perturbation step

n = length(x_est);

% Evaluate measurement size (should be 3)
z0 = H_CSS_function(x_est);
m  = length(z0);

% Pre-allocate Jacobian
H = zeros(m, n);

% Central-difference loop
for i = 1:n
    dx = zeros(n,1);
    dx(i) = eps;

    z_p = H_CSS_function(x_est + dx);
    z_m = H_CSS_function(x_est - dx);

    H(:,i) = (z_p - z_m) / (2*eps);
end

end
