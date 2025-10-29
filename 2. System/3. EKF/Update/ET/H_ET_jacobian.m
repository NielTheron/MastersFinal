%==========================================================================
% Niel Theron
% 02-10-2025
%==========================================================================
% Numerical Jacobian of the Earth Tracker measurement function H_ET_function
% with respect to the state vector x_est using central differences.
%
% Usage:
%   H = H_ET_jacobian(x_est, cat, we, t)
%
% The function returns a 3-by-n Jacobian matrix where n = length(x_est)
%==========================================================================
function H = H_ET_jacobian(x_est, cat, we, t)

% Parameters ==============================================================
eps = 1e-6;             % perturbation step
n   = length(x_est);     % number of state variables

% Evaluate nominal measurement
z0 = H_ET_function(x_est, cat, we, t);  % 3x1

% Pre-allocate Jacobian
H = zeros(3, n);

% Central difference loop
for i = 1:n
    dx = zeros(n,1);
    dx(i) = eps;

    z_p = H_ET_function(x_est + dx, cat, we, t);
    z_m = H_ET_function(x_est - dx, cat, we, t);

    H(:,i) = (z_p - z_m) / (2*eps);
end

end
