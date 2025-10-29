%==========================================================================
% Niel Theron
% 19-06-2025
%==========================================================================
function Image = GenerateSatelliteImage(ax, r_I, v_I, q_O2B, imgWidth, imgHeight, focalLength, pixelSize)
% GenerateSatelliteImage
% Generates a simulated satellite image from camera pose and orientation
%
% Inputs:
%   ax           - Axes handle to render the image
%   r_I          - 3×1 Satellite position in ECI frame
%   v_I          - 3×1 Satellite velocity in ECI frame
%   q_I2B        - 4×1 Quaternion [w; x; y; z] from inertial to body
%   imgWidth     - Image width in pixels
%   imgHeight    - Image height in pixels
%   focalLength  - Camera focal length (m)
%   pixelSize    - Pixel size (m/pixel)
%   rpy          - [roll, pitch, yaw] in radians (camera frame offset)
%
% Output:
%   Image        - RGB image as uint8 matrix

%% Set Up Figure ==========================================================
fig = ancestor(ax, 'figure');
set(fig, 'Visible', 'off');
fig.Units = 'pixels';
fig.Position = [100, 100, imgWidth, imgHeight];
ax.Units = 'normalized';
ax.Position = [0, 0, 1, 1];
set(ax, 'LooseInset', [0 0 0 0]);
axis(ax, 'equal');
axis(ax, 'off');
pbaspect(ax, [imgWidth imgHeight 1]);
daspect(ax, [1 1 1]);

%% Get Rotation Matrices ==================================================

% Get inertial to orbital rotation matrix
R_I2O = RI2O(r_I,v_I);
%---

% Get the inertial to body rotation matrix
R_O2B = quat2rotm(q_O2B.');
%---


%% Set Camera Vectors =====================================================

% Camera view and up direction in the offset frame
camera_view_direction_body = [0; 0; 1];    % Look along +Z in camera
camera_up_direction_body   = [0; -1; 0];   % Up is Y in camera

% Rotate into orbital frame
R_B2O = R_O2B.';
viewDirection_O = R_B2O * camera_view_direction_body;
upVector_O      = R_B2O * camera_up_direction_body;

% Rotate into inertial frame
R_O2I = R_I2O.';
viewDirection_eci = R_O2I * viewDirection_O;
upVector_eci      = R_O2I * upVector_O;

%% Set Camera =============================================================

% Position and target
campos(ax, r_I.');
target_distance = 500;
camtarget(ax, (r_I + viewDirection_eci * target_distance).');

% Projection and up vector
camproj(ax, 'perspective');
camup(ax, upVector_eci.');

% Field of View
fov_v = 2 * atand((imgHeight * pixelSize / 2) / focalLength);
set(ax, 'CameraViewAngleMode', 'manual');
camva(ax, fov_v);

%% Capture Image ==========================================================

drawnow;
frame = getframe(ax);
Image = imresize(frame.cdata, [imgHeight, imgWidth]);
Image = flip(Image,1);
Image = flip(Image,2);
end