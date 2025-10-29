%==========================================================================
% Niel Theron
% 02-10-2025
%==========================================================================
% The Purpose of this function is to compute the numerical Jacobian of the
% GPS measurement function H_GPS with respect to the state vector x_est
% using central differences.
%==========================================================================
function H = H_GPS_jacobian(x_est, we, t)

% Parameters ==============================================================
eps = 1e-6;  % Perturbation step
n   = length(x_est);                    % State vector size
m   = length(H_GPS_function(x_est, we, t));      % Measurement vector size

% Allocate Jacobian
H = zeros(m, n);

% Finite difference loop ==================================================
for i = 1:n
    dx       = zeros(n, 1);
    dx(i)    = eps;
    
    % Forward and backward perturbations
    zhat_p = H_GPS_function(x_est + dx, we, t);
    zhat_m = H_GPS_function(x_est - dx, we, t);
    
    % Central difference
    H(:, i) = (zhat_p - zhat_m) / (2 * eps);
end

end
