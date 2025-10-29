%==========================================================================
% Niel Theron
% 21-05-2025
%==========================================================================
% The purpose of this function is to abstract the GPS, the GPS also has
% drift
%==========================================================================
function [GPS_measurement, GPS_drift_new] = GPS(position, GPS_noise, GPS_drift_prev, drift_rate, we_p, t, dt_p)

% Transform Inertial to ECEF ==============================================
R_I2R = [cos(we_p*t) -sin(we_p*t) 0;
    sin(we_p*t)  cos(we_p*t) 0;
    0            0           1];
GPS_measurement = R_I2R * position;
%==========================================================================

% ===== Noise and Drift Model =============================================

% White noise (GPS measurement noise) in meters
sigma_gps = GPS_noise * randn(3,1);

% Bias drift (Gauss-Markov model)
tau = 300; % correlation time constant [s] for GPS bias (tunable)
phi = exp(-dt_p/tau);
sigma_bias = drift_rate * sqrt(dt_p); % drift_rate in meters/second
GPS_drift_new = phi * GPS_drift_prev + sqrt(1 - phi^2) * sigma_bias * randn(3,1);

% Add noise and drift
GPS_measurement = GPS_measurement + sigma_gps/1000 + GPS_drift_new/1000;

%==========================================================================

%=== Convert to Latitude, Longitude, Altitude (degrees and km) ============
wgs84 = wgs84Ellipsoid("kilometers");
[lat, lon, alt] = ecef2geodetic(wgs84, ...
    GPS_measurement(1), GPS_measurement(2), GPS_measurement(3), "degrees");

GPS_measurement = [lat; lon; alt];
%==========================================================================

end
