%==========================================================================
% Niel Theron
% 19-06-2025
%==========================================================================
% The purpose of this function is to get the ECI position en velocity
% vectors given the LLA vector
%==========================================================================
% Inout:
%   v_L : LLA vector [3x1] [deg,deg,km]
% 
% Output:
%   r_I : Position vector
%   v_I : Velocity vector
%==========================================================================

function [r_I, v_I] = LLA2RV(v_L, Mu, we, t, i)
    % Convert lat/lon to radians
    lat = deg2rad(v_L(1,1));
    lon = deg2rad(v_L(2,1));
    
    % Get inertial position vector
    r_I = AR2I(LLA2ECR(v_L), we, t);
    
    % Local coordinate system in ECEF
    % East unit vector
    east_ecef = [-sin(lon); cos(lon); 0];
    east_unit = east_ecef / norm(east_ecef);
    
    % North unit vector
    north_ecef = [-sin(lat)*cos(lon); -sin(lat)*sin(lon); cos(lat)];
    north_unit = north_ecef / norm(north_ecef);
    
    % Up unit vector (radial outward)
    up_unit = r_I / norm(r_I);
    
    % Calculate velocity direction angle in local horizontal plane
    % From the relationship: cos(i) = cos(lat) * cos(alpha)
    % where alpha is angle from east direction
    i_rad = deg2rad(i);
    cos_alpha = cos(i_rad) / cos(lat);
    
    % Check if inclination is achievable from this latitude
    if abs(cos_alpha) > 1
        error('Inclination %.1f° not achievable from latitude %.1f°', i, rad2deg(lat));
    end
    
    % Calculate alpha (choose the solution that gives prograde motion)
    alpha = acos(cos_alpha);
    
    % Velocity direction in local horizontal plane
    % v_local = cos(alpha) * east + sin(alpha) * north
    v_direction_ecef = cos_alpha * east_unit + sin(alpha) * north_unit;
    
    % Convert to ECI frame
    v_direction_eci = AR2I(v_direction_ecef, we, t);
    
    % Orbital speed for circular orbit
    r_mag = norm(r_I);
    v_mag = sqrt(Mu / r_mag);
    
    % Final velocity vector in ECI
    v_I = v_mag * v_direction_eci;
end
