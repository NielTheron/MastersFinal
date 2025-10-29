%=========================================================================
% Niel Theron
% 02-10-2025
%=========================================================================
% Numerical Jacobian of the Star Tracker measurement function H_ST_function
% with respect to the state vector x_est using central differences.
%
% Usage:
%   H = H_ST_jacobian(x_est)
%
% The function returns a 3-by-d-by-n Jacobian matrix where:
%   - d = number of stars in the catalogue
%   - n = length(x_est)
%=========================================================================
function H = H_ST_jacobian(x_est)

% Parameters ==============================================================
eps = 1e-6; % perturbation step

n = length(x_est);

% Evaluate nominal measurement
Z0 = H_ST_function(x_est);   % 3 x d
[~, d] = size(Z0);

% Pre-allocate Jacobian: 3 x d x n
H = zeros(3, d, n);

% Central-difference loop =================================================
for i = 1:n
    dx = zeros(n,1);
    dx(i) = eps;

    Zp = H_ST_function(x_est + dx); % 3 x d
    Zm = H_ST_function(x_est - dx); % 3 x d

    % Store derivative for this state variable
    H(:,:,i) = (Zp - Zm) / (2*eps);
end

end