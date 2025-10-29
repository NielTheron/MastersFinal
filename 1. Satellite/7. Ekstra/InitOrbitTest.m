%========================================================================
% Niel Theron
% 02-09-2025
%========================================================================

%=== Constants =========================================================
Mu = 3.986e5; 
A_C2B = [0 0 1; 0 1 0; -1 0 0];


%=== Initial Conditions ================================================
lat = 48.858;
lon = 1.66;
alt = 500;
rol = 0;
pit = 0;
yaw = 0;
wx  = 0;
wy  = 0;
wz  = 0;
%======================================================================

[r_I, v_I, q_B2O, w_B2O_B] = InitOrbit(lat, lon, alt, rol, pit, yaw, wx, wy, wz, Mu, A_C2B)

function [r_I, v_I, q_B2O, w_B2O_B] = InitOrbit(lat, lon, alt, rol, pit, yaw, wx, wy, wz, Mu, A_C2B)

%=== Initialise Position ==============================================
[x,y,z] = geodetic2ecef(wgs84Ellipsoid('km'),lat,lon,alt,"degrees");
r_I = [x y z].';
%=== Initialise Velocity ==============================================
u = [-sin(lon), cos(lon), 0].';
u_east = 1/norm(u)*u;

v_mag = sqrt(Mu/norm(r_I));

v_I = v_mag * u_east;

%=== Initialise Attitude ==============================================

A_O2C = eul2rotm([yaw, pit, rol], 'ZYX');

A_O2B = A_C2B * A_O2C;

q_B2O = rotm2quat(A_O2B).';

%=== Initialise Angular Velocity =======================================

w_B2O_B = A_C2B * [wx; wy; wz];

end