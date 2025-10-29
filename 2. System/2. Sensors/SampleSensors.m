function [z_GPS, z_GYR, z_ST, z_CSS, z_MAG] = ...
    SampleSensors(x_true, t, we_p, Mu_p, dt_p,...
    dt_GPS, noise_GPS, enable_GPS, drift_GPS, driftRate_GPS, ...
    dt_GYR, noise_GYR, enable_GYR, drift_GYR, driftRate_GYR, ...
    dt_ST,  noise_ST,  enable_ST, ...
    dt_CSS, noise_CSS, enable_CSS, ...
    dt_MAG, noise_MAG, enable_MAG)

% GPS =====================================================================
j = mod(t,dt_GPS);
if j == 0 && enable_GPS
    z_GPS = GPS(x_true(1:3), noise_GPS, drift_GPS, driftRate_GPS, we_p, t, dt_p);
else
    z_GPS= [0 0 0].';
end
%==========================================================================

% GYR =====================================================================
j = mod(t,dt_GYR);
if j == 0 && enable_GYR
    z_GYR = Gyro(x_true(1:3), x_true(7:10), x_true(11:13), noise_GYR, drift_GYR, driftRate_GYR, Mu_p, dt_p);
else
    z_GYR = [0 0 0].';
end
%==========================================================================

% ST ======================================================================
j = mod(t,dt_ST);
if j == 0 && enable_ST
    z_ST = StarTracker(x_true, noise_ST);
else
    z_ST = zeros(3,20);
end
%==========================================================================

% CSS =====================================================================
j = mod(t,dt_CSS);
if j == 0 && enable_CSS
    z_CSS = CoarseSunSensor(x_true, noise_CSS);
else
    z_CSS = [0 0 0].';
end
%==========================================================================

% MAG ======================================================================
j = mod(t,dt_MAG);
if j == 0 && enable_MAG
    z_MAG = Magnetometer(x_true,noise_MAG,we_p,t);
else
    z_MAG = [0 0 0].';
end
%==========================================================================

end

