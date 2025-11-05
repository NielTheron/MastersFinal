%=========================================================================
% Niel Theron
% 04-09-2025
%========================================================================

function catalogueLLA = Catalogue(features, x_true, focalLength, pixelSize, alpha, Ix, Iy, we_p, t)

%=== T_P2M =============================================================
cx = Ix/2;
cy = Iy/2;

T_P2M = [pixelSize      alpha           0              -cx*pixelSize;
         0             -pixelSize       0               cy*pixelSize;
         0              0              -focalLength     0;
         0              0               0               1];

%f_M = T_P2M * [features; 1; 1]
%---

%=== T_M2C =============================================================

s = 500;
T_M2C = [-s/focalLength   0               0               0;
          0              -s/focalLength   0               0;
          0               0              -s/focalLength   0;
          0               0               0               1];

%f_C = T_M2C * f_M
%----

%=== T_C2B =============================================================

T_C2B = [-1   0   0   0;
          0  -1   0   0;
          0   0   1   0;
          0   0   0   1];

% f_B = T_C2B * f_C
%----

%=== T_B2O =============================================================

R_B2O = quat2rotm(x_true(7:10).').';

T_B2O = [R_B2O          zeros(3,1);
         zeros(1,3)     1];

%f_O = T_B2O * f_B
%----

%=== T_O2I =============================================================

R_I2O = RI2O(x_true(1:3),x_true(4:6));
R_O2I = R_I2O.';
T_O2I_R = [R_O2I         zeros(3,1);
        zeros(1,3)     1];

T_O2I = [R_O2I         x_true(1:3);
        zeros(1,3)     1];

% f_I = T_O2I * f_O
%----

%=== T_I2R ============================================================
theta = we_p * t;
T_I2R = [cos(theta)    -sin(theta)     0 0;
         sin(theta)    cos(theta)      0 0;
         0              0             1 0;
         0              0             0 1];

%=== Ray-Ellipsoid Intersection =========================================

numFeatures = size(features, 2);
catalogueLLA = zeros(3, numFeatures);

% WGS84 ellipsoid parameters
a = 6378.137;  % semi-major axis (km)
b = 6356.752;  % semi-minor axis (km)

% Satellite position in ECI
r_I = x_true(1:3);

for i = 1:numFeatures
    % Convert pixel to direction vector in ECI
    f_P = [features(:,i); 1; 1];
    f_I = T_O2I_R * T_B2O * T_C2B * T_M2C * T_P2M * f_P;
    ray_eci = f_I(1:3) / norm(f_I(1:3));  % Normalize direction
    
    % Ray parameters
    dx = ray_eci(1); dy = ray_eci(2); dz = ray_eci(3);
    x0 = r_I(1); y0 = r_I(2); z0 = r_I(3);
    
    % Quadratic equation coefficients for ray-ellipsoid intersection
    A = (dx^2 + dy^2)/a^2 + (dz^2)/b^2;
    B = 2*((x0*dx + y0*dy)/a^2 + (z0*dz)/b^2);
    C = (x0^2 + y0^2)/a^2 + (z0^2)/b^2 - 1;
    
    discriminant = B^2 - 4*A*C;
    
    % Check for intersection
    if discriminant < 0
        warning("Catalogue: Feature %d has no Earth intersection", i);
        catalogueLLA(:,i) = NaN(3,1);
        continue
    end
    
    % Solve for intersection points
    t1 = (-B - sqrt(discriminant)) / (2*A);
    t2 = (-B + sqrt(discriminant)) / (2*A);
    
    % Select closest positive intersection (forward along ray)
    valid_t = [t1, t2];
    valid_t = valid_t(valid_t > 0);
    
    if isempty(valid_t)
        warning("Catalogue: Feature %d intersection is behind satellite", i);
        catalogueLLA(:,i) = NaN(3,1);
        continue
    end
    
    t_intersect = min(valid_t);
    
    % Compute intersection point in ECI
    P_eci = r_I + t_intersect * ray_eci;
    
    % Convert ECI to ECEF
    P_ecef = T_I2R * [P_eci; 1];
    
    % Convert ECEF to geodetic coordinates
    [lat, lon, alt] = ecef2geodetic(wgs84Ellipsoid('km'), ...
                                    P_ecef(1), P_ecef(2), P_ecef(3));
    
    catalogueLLA(:,i) = [lat; lon; alt];
end

end
