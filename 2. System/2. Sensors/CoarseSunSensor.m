%==========================================================================
% Niel Theron
% 25-09-2025
%==========================================================================
% The purpose of this function is to create the sun vector measurement in
% the body frame
%=========================================================================
function Sun_B = CoarseSunSensor(x_true, CSS_noise)

%=== Sun Vector ===========================================================
Sun_I = [150e6, 0, 0, 1].';            % Sun vector in inertial
%==========================================================================

%=== T_I2O ================================================================
R_I2O = RI2O(x_true(1:3),x_true(4:6));

T_I2O = [R_I2O         -R_I2O*x_true(1:3);
        zeros(1,3)     1];

%==========================================================================

%=== T_O2B ================================================================
R_O2B = quat2rotm(x_true(7:10).');

T_O2B = [R_O2B          zeros(3,1);
         zeros(1,3)     1];

%==========================================================================

%=== Sun_B Vector =========================================================

Sun_B_true = T_O2B * T_I2O * Sun_I;
Sun_B_true = Sun_B_true(1:3);
Sun_B_true = Sun_B_true/norm(Sun_B_true);

%==========================================================================

%=== CSS Sensor directions ================================================
css_dirs = [ 1  0  0;   % +X
            -1  0  0;   % -X
             0  1  0;   % +Y
             0 -1  0;   % -Y
             0  0  1;   % +Z
             0  0 -1];  % -Z
%==========================================================================

%=== Simulate CSS readings (cosine response + max(0)) =====================
css_readings = max(0, css_dirs * Sun_B_true);  % 6x1 vector of readings
%==========================================================================

%=== Add noise to each CSS reading ========================================
sigma = CSS_noise/100;    
css_readings = css_readings + sigma * randn(size(css_readings));
css_readings = max(css_readings, 0); % Clamp negatives
%==========================================================================

%=== Reconstruct sun vector estimate from readings ========================
% Weighted sum of sensor directions
Sun_B_est = css_dirs' * css_readings;    % 3x1
Sun_B = Sun_B_est / norm(Sun_B_est); % Normalize to unit vector
%==========================================================================

end