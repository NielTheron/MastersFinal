%==========================================================================
% Niel Theron
% 02-10-2025
%==========================================================================
% The Purpose of this function is to is to create an estimated measurement
% measurement from the estimated state to compare to the real measurement
%==========================================================================
function zhat_GPS = H_GPS_function(x_est, we, t)

% Transform Inertial to ECEF ==============================================
R_I2R = [cos(we*t) -sin(we*t) 0;
         sin(we*t)  cos(we*t) 0;
         0            0       1];

zhat_GPS = R_I2R * x_est(1:3);
%==========================================================================

%=== Convert to Latitude, Longitude, Altitude (degrees and km) ============
wgs84 = wgs84Ellipsoid("kilometers");
[lat, lon, alt] = ecef2geodetic(wgs84, ...
    zhat_GPS(1), zhat_GPS(2), zhat_GPS(3), "degrees");

zhat_GPS = [lat; lon; alt];
%==========================================================================

end