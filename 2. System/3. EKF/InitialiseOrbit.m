function [r_I, v_I, q_O2B, w_O2B_B] ...
 = InitialiseOrbit( ...
 lat, lon, alt, ...
 rol, pit, yaw,...
 wx, wy, wz, ...
 Mu, we, t, i)
%==========================================================================
% InitialiseOrbitAligned
%==========================================================================
% Purpose: Initialize satellite orbit with body frame aligned to orbital frame
%
% Inputs:
%   lat, lon, alt - Initial orbital position
%   rol, pit, yaw - Attitude offsets from orbital frame IN CAMERA FRAME (deg)
%   wx, wy, wz - Body rates relative to orbital frame IN CAMERA FRAME (deg/s)
%
% Outputs:
%   r_I            - Position vector (km) [3x1]
%   v_I            - Velocity vector (km/s) [3x1] 
%   q_I2B          - Inertial-to-Body quaternion [4x1] [qw, qx, qy, qz]
%   w_I2B_B        - Body angular velocity relative to inertial frame (rad/s) [3x1]
%==========================================================================

%=== Constant ============================================================
R_C2B = [0 0 -1; 0 1 0; 1 0 0];
R_O2C_D = [0 0 1; 0 1 0; -1 0 0];

%=== Initialise Position and Velocity ====================================
[r_I, v_I] = LLA2RV([lat; lon; alt],Mu,we,t,i);

%=== Initialise Attitude =================================================
R_O2C = eul2rotm(deg2rad([yaw, pit, rol]),"ZYX") * R_O2C_D;
R_O2B = R_C2B * R_O2C;
q_O2B = rotm2quat(R_O2B).';

if q_O2B(1) < 0
    q_O2B = -q_O2B;
end

%=== Initialise Angular velocity =========================================
w_O2B_B = R_C2B * deg2rad([wx; wy; wz]);

%=========================================================================
end