%==========================================================================
% Niel Theron & Google's Gemini
% 16-10-2025
% Final Version for MATLAB Compatibility
%==========================================================================
function Image = GenerateSatelliteImage_RayTrace2(r_I, v_I, q_O2B, imgWidth, imgHeight, focalLength, pixelSize, sourceMapFile)
% GenerateSatelliteImage_RayTrace
% Generates a simulated satellite image using ray-tracing.
% This version loads the entire source map into memory once to ensure
% compatibility with older MATLAB versions.

%% 1. Get Rotation and Load Source Map
% Get rotation from Body frame to Inertial (ECI) frame
R_I2O = RI2O(r_I, v_I);
R_O2B = quat2rotm(q_O2B.');
R_B2I = (R_I2O' * R_O2B');

% Load the entire GeoTIFF and its reference into memory.
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
rays_body_flat = reshape(rays_body_normalized, [], 3)';
rays_eci_flat = R_B2I * rays_body_flat;

%% 4. Perform Ray-Tracing for Each Pixel
R_earth_km = 6378.137; % WGS84 Equatorial radius in km
outputImage = zeros(imgHeight, imgWidth, 3, 'uint8');

% Get the row/col for each pixel in the output image
[pixel_rows, pixel_cols] = ind2sub([imgHeight, imgWidth], 1:(imgHeight*imgWidth));

% Use parfor for a significant speedup if you have the Parallel Computing Toolbox
for idx = 1:(imgHeight * imgWidth)
    d = rays_eci_flat(:, idx); % Direction vector for this pixel
    
    % Solve quadratic equation for ray-sphere intersection
    b = 2 * dot(d, r_I);
    c_quad = dot(r_I, r_I) - R_earth_km^2;
    delta = b^2 - 4*c_quad; % 'a' is 1 since d is a unit vector
    
    if delta >= 0
        t = (-b - sqrt(delta)) / 2; % Smallest positive distance
        if t > 0
            intersect_eci = r_I + t * d;
            
            % Convert ECI intersection point to latitude and longitude
            lla = ecef2lla(intersect_eci' * 1000); % ecef2lla needs meters
            lat = lla(1);
            lon = lla(2);
            
            % Convert lat/lon to pixel coordinates of the source map
            [col, row] = geographicToIntrinsic(R, lat, lon);
            row = round(row);
            col = round(col);
            
            % Check if the point is within the bounds of our source map
            if row >= 1 && row <= R.RasterSize(1) && col >= 1 && col <= R.RasterSize(2)
                % Select only the first 3 channels (RGB) to ignore the alpha channel.
                pixel_color = sourceImage(row, col, 1:3);
                
                % Place the color in the correct output pixel
                outputImage(pixel_rows(idx), pixel_cols(idx), :) = pixel_color;
            end
        end
    end
end

Image = outputImage;
end