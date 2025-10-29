%========================================================================
% MATLAB Orbit Initialisation and Visualisation Script
%
% Niel Theron (constants and initial conditions)
% Gemini (visualisation code)
% 02-09-2025
%
% Purpose:
% This script first calls the InitOrbit function to calculate the
% initial state of a satellite. It then creates an interactive 3D plot
% showing the Earth, the satellite's position, its calculated orbit
% path, its velocity vector, and its body reference frame (orientation).
%========================================================================

clear; clc; close all;

%=== Constants =========================================================
Mu = 3.986e5; % Earth's gravitational parameter (km^3/s^2)
we = 7.292115e-5; % Earth's rotation rate (rad/s)
Re = 6378.137; % Earth's equatorial radius (km)

% Transformation matrix from Camera Frame to Body Frame.
% This defines how the camera is mounted on the satellite.
% X_body = Z_cam, Y_body = Y_cam, Z_body = -X_cam
A_C2B = [0 0 1; 0 1 0; -1 0 0];

%=== Initial Conditions ================================================
% These values define the satellite's starting point and orientation.
lat = 0; % Latitude (deg) -> Stellenbosch, South Africa
lon = 0;  % Longitude (deg) -> Stellenbosch, South Africa
alt = 5000;    % Altitude (km)
rol = 10;     % Roll offset from orbital frame (deg)
pit = 0;     % Pitch offset from orbital frame (deg)
yaw = 0;     % Yaw offset from orbital frame (deg)
wx  = 0;      % Body rates (rad/s)
wy  = 0;
wz  = 0;

% FIX 5: Added orbit_direction to allow for more flexible initialisation.
% Options: 'prograde', 'retrograde', 'polar-north'
orbit_direction = 'prograde'; 

%=== Run Initialisation Function =======================================
[r_I, v_I, q_O2B, w_B2O_B] = InitOrbit(lat, lon, alt, rol, pit, yaw, wx, wy, wz, Mu, A_C2B, orbit_direction);

fprintf('Initialisation Complete:\n');
fprintf('Position [r_I] = [%.2f, %.2f, %.2f] km\n', r_I);
fprintf('Velocity [v_I] = [%.2f, %.2f, %.2f] km/s\n', v_I);
fprintf('Quaternion [q_O2B] = [%.3f, %.3f, %.3f, %.3f]\n', q_O2B);

%=== Propagate Orbit for Visualisation =================================
% We will propagate the orbit for one full period to plot the path.
% NOTE: This propagator uses a simple two-body model in an inertial
% frame. Your initial state is calculated in an ECEF frame. For this
% short-term visualisation, we neglect the Earth's rotation, which is a
% reasonable simplification.

period = 2 * pi * sqrt(norm(r_I)^3 / Mu); % Orbital period in seconds
t_span = [0, period]; % Time span for one orbit

% Options for the ODE solver
options = odeset('RelTol', 1e-8, 'AbsTol', 1e-8);

% State vector is [x; y; z; vx; vy; vz]
initial_state = [r_I; v_I];

% Call the ODE solver to get the trajectory
[~, state_history] = ode45(@two_body_propagator, t_span, initial_state, options, Mu);
orbit_path = state_history(:, 1:3); % Extract position history

%=== Create Visualisation ==============================================
fprintf('Generating visualisation...\n');
figure('Name', 'Satellite Orbit Visualisation', 'NumberTitle', 'off', 'Position', [100 100 1000 800]);
ax = axes('Color', 'k'); % Use black background for space
hold(ax, 'on');

%-- Plot the Earth ----------------------------------------------------
% Create a sphere and map an Earth texture onto it.
[x, y, z] = sphere(50);
earth = surf(ax, x*Re, y*Re, -z*Re);
set(earth, 'FaceColor', 'texturemap', 'CData', imread('earth.jpg'), 'EdgeColor', 'none');
set(ax, 'Color', 'k'); % Black background
axis equal;
grid on;
title(ax, 'Satellite Initial State and Orbit Path');
xlabel(ax, 'X_{ECEF} (km)');
ylabel(ax, 'Y_{ECEF} (km)');
zlabel(ax, 'Z_{ECEF} (km)');
view(3);

%-- Plot the Orbit Path -----------------------------------------------
plot3(ax, orbit_path(:,1), orbit_path(:,2), orbit_path(:,3), 'c--', 'LineWidth', 1.5, 'DisplayName', 'Orbit Path');

%-- Plot Satellite Position -------------------------------------------
plot3(ax, r_I(1), r_I(2), r_I(3), 'yo', 'MarkerSize', 10, 'MarkerFaceColor', 'y', 'DisplayName', 'Satellite');

%-- Plot Velocity and Body Frame Axes ---------------------------------
% 1. Calculate the rotation from Inertial/ECEF to the Orbital Frame (A_I2O)
% This defines a standard Nadir-pointing orbital frame (VVLH).
% Z-axis points to Earth's center (-r_hat).
% Y-axis is opposite the orbit normal (-h_hat).
% X-axis is in the direction of motion (theta_hat), completing the right-hand rule.
r_hat = r_I / norm(r_I);
h_vec = cross(r_I, v_I);
h_hat = h_vec / norm(h_vec);
theta_hat = cross(h_hat, r_hat);

x_orb = theta_hat;
y_orb = -h_hat;
z_orb = -r_hat;

% The rotation matrix from Orbital to Inertial has the orbital axes as its columns.
A_O2I = [x_orb, y_orb, z_orb];
% The desired rotation matrix from Inertial to Orbital is the transpose of this.
A_I2O = A_O2I';

% 2. Get the rotation from Orbital to Body Frame (A_O2B) from the quaternion
A_O2B = quat2rotm(q_O2B);

% 3. Combine them to get the final rotation from Inertial to Body (A_I2B)
A_I2B = A_O2B * A_I2O;

% Define standard body axes vectors
body_x = [1; 0; 0];
body_y = [0; 1; 0];
body_z = [0; 0; 1];

% Rotate body axes into the inertial frame for plotting
body_x_I = A_I2B' * body_x; % Use transpose for vector transformation
body_y_I = A_I2B' * body_y;
body_z_I = A_I2B' * body_z;

% Scale vectors for better visibility
axis_length = 0.2 * norm(r_I);
vel_length = axis_length;

% Plot Body Frame Axes using quiver3
quiver3(ax, r_I(1), r_I(2), r_I(3), body_x_I(1), body_x_I(2), body_x_I(3), axis_length, 'r', 'LineWidth', 2, 'DisplayName', 'Body X-axis');
quiver3(ax, r_I(1), r_I(2), r_I(3), body_y_I(1), body_y_I(2), body_y_I(3), axis_length, 'g', 'LineWidth', 2, 'DisplayName', 'Body Y-axis');
quiver3(ax, r_I(1), r_I(2), r_I(3), body_z_I(1), body_z_I(2), body_z_I(3), axis_length, 'b', 'LineWidth', 2, 'DisplayName', 'Body Z-axis');

% Plot Velocity Vector
v_norm = v_I / norm(v_I);
quiver3(ax, r_I(1), r_I(2), r_I(3), v_norm(1), v_norm(2), v_norm(3), vel_length, 'm', 'LineWidth', 2, 'DisplayName', 'Velocity');

%-- Final Touches -----------------------------------------------------
legend(ax, 'show', 'TextColor', 'w');
rotate3d on;
disp('Visualisation generated. Rotate the plot with your mouse.');

%======================================================================
%                        LOCAL FUNCTIONS
%======================================================================

function [r_I, v_I, q_O2B, w_B2O_B] = InitOrbit(lat, lon, alt, rol, pit, yaw, wx, wy, wz, Mu, A_C2B, orbit_direction)
% Purpose: Initialize a satellite's state in the ECEF frame.
% Inputs:
%   lat, lon, alt - Geodetic coordinates (deg, deg, km)
%   rol, pit, yaw - Euler angles defining rotation from Orbital to Camera frame (deg)
%   wx, wy, wz    - Angular rates in the Camera frame (rad/s)
%   Mu            - Earth's gravitational parameter (km^3/s^2)
%   A_C2B         - Rotation matrix from Camera to Body frame
%   orbit_direction - String specifying orbit type: 'prograde', 'retrograde', 'polar-north'
% Outputs:
%   r_I           - Position vector in ECEF (km) [3x1]
%   v_I           - Velocity vector in ECEF (km/s) [3x1]
%   q_O2B         - Quaternion from Orbital to Body frame [qw, qx, qy, qz]
%   w_B2O_B       - Angular velocity of Body wrt Orbital, in Body frame (rad/s)

%=== Initialise Position (Geodetic to ECEF) ===========================
[x, y, z] = geodetic2ecef(wgs84Ellipsoid('km'), lat, lon, alt, 'degrees');
r_I = [x; y; z];

%=== Initialise Velocity (Assuming circular orbit) =====================
v_mag = sqrt(Mu/norm(r_I));
z_axis_I = [0; 0; 1]; % Earth's rotation axis

switch lower(orbit_direction)
    case 'prograde'
        % Velocity vector is eastward, parallel to the equator.
        v_dir = cross(z_axis_I, r_I);
    case 'retrograde'
        % Velocity vector is westward.
        v_dir = cross(r_I, z_axis_I);
    case 'polar-north'
        % Velocity vector points towards the local North.
        up_hat = r_I / norm(r_I);
        east_dir = cross(z_axis_I, r_I);
        east_hat = east_dir / norm(east_dir);
        north_hat = cross(up_hat, east_hat);
        v_dir = north_hat;
    otherwise
        error("Invalid 'orbit_direction'. Use 'prograde', 'retrograde', or 'polar-north'.");
end

v_hat = v_dir / norm(v_dir);
v_I = v_mag * v_hat;

%=== Initialise Attitude ===============================================
% 1. Rotation from Orbital Frame to Camera Frame (based on RPY inputs)
%    Using 'ZYX' convention for Yaw-Pitch-Roll.
A_O2C = eul2rotm(deg2rad([yaw, pit, rol]), 'ZYX');

% 2. Rotation from Orbital Frame to Body Frame (chaining rotations)
A_O2B = A_C2B * A_O2C;

% 3. Convert final rotation matrix to quaternion
q_O2B = rotm2quat(A_O2B);

%=== Initialise Angular Velocity =======================================
% This is the angular velocity of the Body frame relative to the Orbital
% frame, expressed in the Body frame.
w_B2O_B = A_C2B * [wx; wy; wz];
end

function d_state = two_body_propagator(t, state, Mu)
% Propagates the state vector [r; v] using the two-body equation of motion.
r = state(1:3);
v = state(4:6);

r_norm = norm(r);

% Acceleration a = -Mu * r / |r|^3
a = -Mu * r / (r_norm^3);

d_state = [v; a];
end


