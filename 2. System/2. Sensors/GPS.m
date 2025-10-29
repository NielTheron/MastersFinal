%==========================================================================
% Niel Theron
% 21-05-2025
%==========================================================================
% The purpose of this function is to abstract the GPS, the GPS also has
% drift
%==========================================================================
function [GPS_measurement, GPS_drift_new] = GPS(position, GPS_noise, ...
    GPS_drift_prev, drift_rate, we_p, t, dt_p, tau_bias)
%==========================================================================
% GPS Sensor Simulation with Gauss-Markov Bias Drift
%
% Inputs:
%   position        - [3x1] ECI position [km]
%   GPS_noise       - Standard deviation of white noise [m]
%   GPS_drift_prev  - [3x1] Previous bias state [m]
%   drift_sigma     - Standard deviation of bias steady-state [m]
%   we_p            - Earth rotation rate [rad/s]
%   t               - Current time [s]
%   dt_p            - Time step [s]
%   tau_bias        - (Optional) Correlation time for bias [s], default=300
%
% Outputs:
%   GPS_measurement - [3x1] [latitude(deg), longitude(deg), altitude(km)]
%   GPS_drift_new   - [3x1] Updated bias state [m]
%==========================================================================

if nargin < 8
    tau_bias = 3600; % Default correlation time [s]
end

% Transform ECI to ECEF ==================================================
theta = we_p * t;
c = cos(theta);
s = sin(theta);
R_I2R = [c -s 0;
         s  c 0;
         0  0 1];

GPS_position_ecef_km = R_I2R * position; % [km]
%=========================================================================

% Noise and Drift Model ==================================================

white_noise_m = GPS_noise * randn(3,1);
RW_coeff_per_sec = drift_rate / sqrt(3600); % [m/√s]
q_bias = RW_coeff_per_sec^2;
sigma_bias_ss = sqrt(q_bias * tau_bias / 2);
phi = exp(-dt_p / tau_bias);
sigma_drive = sigma_bias_ss * sqrt(1 - phi^2);

GPS_drift_new = phi * GPS_drift_prev + sigma_drive * randn(3,1); % [m]
total_error_m = white_noise_m + GPS_drift_new; % [m]
GPS_position_ecef_km = GPS_position_ecef_km + total_error_m / 1000;
%=========================================================================

% Convert to Geodetic Coordinates ========================================
wgs84 = wgs84Ellipsoid("kilometers");
[lat, lon, alt] = ecef2geodetic(wgs84, ...
    GPS_position_ecef_km(1), ...
    GPS_position_ecef_km(2), ...
    GPS_position_ecef_km(3), ...
    "degrees");

GPS_measurement = [lat; lon; alt]; % [deg, deg, km]
%=========================================================================

end
