%==========================================================================
% Niel Theron
% 17-07-2025
%==========================================================================

%% Load Functions =========================================================

% This adds all the functions to the compile path
LoadPath
warning('off');
CleanFolders;
clearvars;
clc;
close all;
%---

%==========================================================================

%% Simualtion Paramters ===================================================

% Simulation Parameters
st      = 40;                            % Simulation time (s)
dt_p    = 0.1;                            % Sample rate (s)
n_s     = round(st/dt_p);               % Number of samples                      
%---

%==========================================================================

%% Initialise Plant =======================================================

% Plant Variables
n_x     = 13;                           % Number of states
x_true  = zeros(n_x,n_s);               % Initialise plant states 
%---

% Plant Constants
Mu_p = 3.986e5;                         % Gravitational parameter (km3/s2)
Re_p = 6.378e3;                         % Radius of Earth (km)
J2_p = 1.082e-3;                        % J2 parameter
I_p  = [1 1 1].';                       % Moment of inertia (kg*m2)
we_p = 0; %7.292e-5;                        % Rotational speed of earth (rad/s)
%---

% Initial States
lat_p       = -33.90;                      % Lattitude (deg)
lon_p       = 18.41;                     % Longitude (deg)
alt_p       = 500;                      % Altitude (km)
rotx_p      = 0;                        % Satellite roll  Offset B/O (deg)
roty_p      = 0;                        % Satellite Pitch Offset B/O (deg)
rotz_p      = 0;                        % Satellite Yaw   Offset B/O (deg)
wx_p        = 0;                        % Satellite Roll  rate B/O (deg/s)
wy_p        = 0.0011;                        % Satellite Pitch rate B/O (deg/s) 
wz_p        = 0;                        % Satellite Yaw rate B/O (deg/s)
inc_p       = -33.90;                   % Initial Inclination (s)
%---

% Initialize orbit
[r_p, v_p, q_O2B_p, w_O2B_p] = ...
InitialiseOrbit( ...
lat_p, lon_p, alt_p, ...
rotx_p, roty_p, rotz_p, ...
wx_p, wy_p, wz_p, ...
Mu_p, we_p, dt_p, inc_p);
%---

% Set initial true state
x_true(:,1) = [r_p; v_p; q_O2B_p; w_O2B_p];
%---

%==========================================================================

%% Initialise Camera ======================================================

% Render Environement
[imgPatch, R] = readgeoraster('CapeStrip5.tif');
ax = RenderEarth(imgPatch,R);
%---

% Camera Variables
imgWidth_cam        = 720;                  % Along track image width (pixels)
imgHeight_cam       = 720;                  % Cross-track image height (pixels)
focalLength_cam     = 0.58;                 % Focal length (m)
pixelSize_cam       = 17.4e-6;              % Pixel size (m)
%---

% Initialise Satellite Images
% numImages = size(x_true, 2);
% satelliteImages = zeros(imgHeight_cam, imgWidth_cam, 3, numImages, 'uint8');
%---

%==========================================================================

%% Initialise Simulation ==================================================

% Progress bar
fig = uifigure('Name','Simulation Progress');
d = uiprogressdlg(fig, 'Title','Running Simulation', ...
    'Message','Initializing...', 'Indeterminate','off');
startTime = tic;
%---

%==========================================================================
%% Run Simulation =========================================================
for r = 1:n_s-1

    % Variables -----------------------------------------------------------
    t = r*dt_p;
    %----------------------------------------------------------------------

    % Plant ---------------------------------------------------------------
    x_true(:,r+1) = Plant(x_true(:,r), dt_p, I_p, Mu_p, Re_p, J2_p);
    %----------------------------------------------------------------------

    % Rotate Earth
    % earth_rotation_deg = rad2deg(we_p * dt_p);
    % RotateEarth(ax, earth_rotation_deg);
    %----------------------------------

    % Image generator -----------------------------------------------------
    satelliteImages(:,:,:,r) = GenerateSatelliteImage(ax, x_true(1:3,r), x_true(4:6,r), x_true(7:10,r), imgWidth_cam, imgHeight_cam, focalLength_cam, pixelSize_cam);
    SaveSatelliteImages(satelliteImages(:,:,:,r),r);
    % ----------------------------------------------------------------------

    % Progress bar --------------------------------------------------------
    elapsedTime = toc(startTime);
    progress = r / (n_s-1);
    estTotalTime = elapsedTime / progress;
    estRemaining = estTotalTime - elapsedTime;
    d.Value = progress;
    d.Message = sprintf('Elapsed: %.2fs | %d%% | Est. remaining: %.2fs | Sample: %d | Footage Time: %.2fs | Time per sample: %.2fs', ...
        elapsedTime, round(progress*100), estRemaining,r+1,r*dt_p,elapsedTime/r);
    %----------------------------------------------------------------------
end

%==========================================================================

%% Save Output as a struct

dataset = struct();
dataset.poses = x_true;                         % Ground truth poses
dataset.images = satelliteImages;               % Generated images
dataset.simulationTime = st;
dataset.sampleTime = dt_p;

% Physical parameters
dataset.metadata.physics.Mu = Mu_p;             % Gravitational parameter (km3/s2)
dataset.metadata.physics.Re = Re_p;             % Radius of Earth (km)
dataset.metadata.physics.J2 = J2_p;             % J2 parameter
dataset.metadata.physics.I = I_p;               % Moment of inertia (kg*m2)
dataset.metadata.physics.we = we_p;             % Rotational speed of earth (rad/s)

% Initial conditions
dataset.metadata.initial.lat  = lat_p;           % Latitude (deg)
dataset.metadata.initial.lon  = lon_p;           % Longitude (deg)
dataset.metadata.initial.alt  = alt_p;           % Altitude (km)
dataset.metadata.initial.rotx = rotx_p;          % Camera roll Offset B/O (deg)
dataset.metadata.initial.roty = roty_p;          % Camera Pitch Offset B/O (deg)
dataset.metadata.initial.rotz = rotz_p;          % Camera Yaw Offset B/O (deg)
dataset.metadata.initial.wx   = wx_p;            % Camera Roll rate B/O (deg/s)
dataset.metadata.initial.wy   = wy_p;            % Camera Pitch rate B/O (deg/s)
dataset.metadata.initial.wz   = wz_p;            % Camera Yaw rate B/O (deg/s)
dataset.metadata.initial.inc  = inc_p;           % Initial time (s)

% Camera parameters
dataset.metadata.camera.imgWidth = imgWidth_cam;
dataset.metadata.camera.imgHeight = imgHeight_cam;
dataset.metadata.camera.focalLength = focalLength_cam;
dataset.metadata.camera.pixelSize = pixelSize_cam;

% Dataset info
dataset.metadata.info.dateCreated = datetime('now');
dataset.metadata.info.description = 'Satellite pose estimation dataset';

timestamp = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
filename = sprintf('Dataset_%s.mat', timestamp);
filepath = fullfile('C:\Users\Niel\Desktop\1. Masters\1. Satellite\4. Datasets', filename);

% Save with timestamped filename
save(filepath, 'dataset');
%==========================================================================














