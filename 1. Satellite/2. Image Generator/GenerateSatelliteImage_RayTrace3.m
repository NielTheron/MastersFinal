%==========================================================================
% Niel Theron & Claude (Anthropic)
% 29-10-2025
% Corrected Version with Earth Rotation Handling
%==========================================================================
function Image = GenerateSatelliteImage_RayTrace(r_I, v_I, q_O2B, imgWidth, imgHeight, focalLength, pixelSize, sourceMapFile, t)
% GenerateSatelliteImage_RayTrace_Corrected
% Generates a simulated satellite image using ray-tracing with an 
% ellipsoidal Earth model and PROPER Earth rotation handling.
%
% CRITICAL FIX: Now properly transforms ECI → ECEF before converting to LLA
%
% INPUTS:
%   r_I           - Satellite position in ECI frame [km] (3x1)
%   v_I           - Satellite velocity in ECI frame [km/s] (3x1)
%   q_O2B         - Quaternion from Orbital to Body frame (4x1)
%   imgWidth      - Image width in pixels
%   imgHeight     - Image height in pixels
%   focalLength   - Camera focal length [mm]
%   pixelSize     - Pixel size [mm]
%   sourceMapFile - Path to GeoTIFF source map
%   t             - Time since epoch [s] (for Earth rotation)
%
% OUTPUT:
%   Image         - Generated RGB image (imgHeight x imgWidth x 3)

%% 1. Get Rotation and Load Source Map
% Get rotation from Body frame to Inertial (ECI) frame
R_I2O = RI2O(r_I, v_I);
R_O2B = quat2rotm(q_O2B.');
R_B2I = (R_I2O' * R_O2B');

% Load the entire GeoTIFF and its reference into memory
try
    sourceImage = readgeoraster(sourceMapFile);
    R = georasterinfo(sourceMapFile).RasterReference;
catch E
    error('Could not read the source map file: %s. Make sure it is a valid GeoTIFF and the Mapping Toolbox is installed.', sourceMapFile);
end

%% 2. Create Grid of Rays in Body Frame
[cols, rows] = meshgrid(1:imgWidth, 1:imgHeight);
X_cam = -(cols - imgWidth/2 - 0.5) * pixelSize;
Y_cam = -(rows - imgHeight/2 - 0.5) * pixelSize;
Z_cam = ones(imgHeight, imgWidth) * focalLength;

rays_body = cat(3, X_cam, Y_cam, Z_cam);
norms = sqrt(sum(rays_body.^2, 3));
rays_body_normalized = rays_body ./ norms;

%% 3. Transform Rays to ECI Frame
rays_body_flat = reshape(rays_body_normalized, [], 3)';  % 3 x N
R_O2B = quat2rotm(q_O2B.' ./ norm(q_O2B));  % ensure normalized
R_I2O = RI2O(r_I, v_I);
R_B2I = (R_I2O' * R_O2B');                   % Body → Inertial
rays_eci_flat = R_B2I * rays_body_flat;      % 3 x N

%% 4. Vectorized Ray-Ellipsoid Intersection
% WGS84 Ellipsoid parameters (in km)
a = 6378.137;      % Semi-major axis (equatorial radius)
b = 6356.7523142;  % Semi-minor axis (polar radius)

% Extract ray components
dx = rays_eci_flat(1, :);
dy = rays_eci_flat(2, :);
dz = rays_eci_flat(3, :);

% Extract satellite position components
x0 = r_I(1);
y0 = r_I(2);
z0 = r_I(3);

% Compute quadratic coefficients for ellipsoid intersection
A_coeff = (dx.^2 + dy.^2) / a^2 + dz.^2 / b^2;
B_coeff = 2 * ((x0*dx + y0*dy) / a^2 + z0*dz / b^2);
C_coeff = (x0^2 + y0^2) / a^2 + z0^2 / b^2 - 1;

% Compute discriminant
delta = B_coeff.^2 - 4 * A_coeff .* C_coeff;

% Find intersection distances
t_intersect = inf(size(delta));
valid_intersection = delta >= 0;

if any(valid_intersection)
    sqrt_delta = sqrt(delta(valid_intersection));
    t1 = (-B_coeff(valid_intersection) - sqrt_delta) ./ (2 * A_coeff(valid_intersection));
    t2 = (-B_coeff(valid_intersection) + sqrt_delta) ./ (2 * A_coeff(valid_intersection));
    
    t_valid = t1;
    t_valid(t1 <= 0) = t2(t1 <= 0);
    t_valid(t_valid <= 0) = inf;
    
    t_intersect(valid_intersection) = t_valid;
end

%% 5. Compute Intersection Points with Earth Rotation (keep using LLA)
outputImage = zeros(imgHeight, imgWidth, 3, 'uint8');

valid_pixels = isfinite(t_intersect);
if any(valid_pixels)
    [pixel_rows, pixel_cols] = ind2sub([imgHeight, imgWidth], find(valid_pixels));

    valid_rays = rays_eci_flat(:, valid_pixels);
    valid_t = t_intersect(valid_pixels);
    intersect_eci = r_I + valid_rays .* valid_t;  % 3 x n_valid

    % Earth rotation
    we = 7.2921159e-5;  % rad/s
    theta = we * t;
    R_I2R = [cos(theta)  sin(theta)  0;
            -sin(theta)  cos(theta)  0;
             0           0           1];

    intersect_ecef = R_I2R * intersect_eci;   % 3 x n_valid

    % Convert to LLA
    intersect_ecef_m = intersect_ecef' * 1000;  % m
    lla = ecef2lla(intersect_ecef_m);           % [lat, lon, alt]

    % Map to your local GeoTIFF
    [map_cols, map_rows] = geographicToIntrinsic(R, lla(:,1), lla(:,2));
    map_rows = round(map_rows);
    map_cols = round(map_cols);

    % Bounds and rendering
    in_bounds = (map_rows >= 1) & (map_rows <= R.RasterSize(1)) & ...
                (map_cols >= 1) & (map_cols <= R.RasterSize(2));

    if any(in_bounds)
        out_idx = sub2ind([imgHeight, imgWidth], ...
                          pixel_rows(in_bounds), pixel_cols(in_bounds));
        for c = 1:3
            src_idx = sub2ind(size(sourceImage), ...
                              map_rows(in_bounds), map_cols(in_bounds), c);
            tmp = outputImage(:,:,c);
            tmp(out_idx) = sourceImage(src_idx);
            outputImage(:,:,c) = tmp;
        end
    end
end

Image = outputImage;

end