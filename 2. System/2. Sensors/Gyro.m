%==========================================================================
% Niel Theron
% 25-09-2025
%==========================================================================
% The purpose of this function is to create the Gyroscope measurement
%==========================================================================
function [GYR_measurement, GYR_drift_new] = Gyro(pos, att, angV, GYR_noise, GYR_drift_prev, drift_rate, Mu_p, dt_p)

% Calculate WO/I (orbital angular velocity in orbital frame)
w_I2O_in_O = sqrt(Mu_p/(norm(pos)^3)) * [0; 1; 0];

% Extract individual quaternion components for clarity
qs = att(1);  % Scalar component
qx = att(2);  % X vector component
qy = att(3);  % Y vector component
qz = att(4);  % Z vector component

% Construct Rotation Matrix from ECI to Body Frame
R_O2B = [
    1 - 2*(qy^2 + qz^2),  2*(qx*qy - qs*qz),  2*(qx*qz + qs*qy);
    2*(qx*qy + qs*qz),   1 - 2*(qx^2 + qz^2),  2*(qy*qz - qs*qx);
    2*(qx*qz - qs*qy),   2*(qy*qz + qs*qx),   1 - 2*(qx^2 + qy^2)
    ];

% Get Total Angular Velocity in Body Frame
w_I2O_in_B = R_O2B * w_I2O_in_O;
w_I2B_in_B = w_I2O_in_B + angV;

% ===== Noise and Drift Model =====

% White noise (gyro measurement noise)
sigma_gyr = deg2rad(GYR_noise) * randn(3,1);  % in rad/s

% Bias drift (Gauss-Markov model for bounded drift)
tau = 500;  % correlation time constant [s] (tune as needed)
phi = exp(-dt_p/tau);
sigma_bias = deg2rad(drift_rate)*sqrt(dt_p); % steady-state std (rad/s)
GYR_drift_new = phi * GYR_drift_prev + sqrt(1 - phi^2) * sigma_bias * randn(3,1);

% Final measurement
GYR_measurement = w_I2B_in_B + sigma_gyr + GYR_drift_new;

end
