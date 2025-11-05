%==========================================================================
% Niel Theron
% 17-07-2025
%==========================================================================

%% Load Functions =========================================================
clc;
clearvars;
close all;
LoadPath;       % This adds all the functions to the compile path
warning('off');
CleanFolders;
%---

%==========================================================================

%% Simulation Paramters ===================================================
st      = 120;       % Simulation time (s)
dt_p    = 1/60;      % Sample rate (s)
n_s     = round(st/dt_p); % Number of samples                      
%---

%==========================================================================

%% Initialise Plant =======================================================
% Plant Variables
n_x     = 13;                           % Number of states
x_true  = zeros(n_x, n_s);              % Initialise plant states in memory
%---

% Plant Constants
Mu_p = 3.986e5;                         % Gravitational parameter (km3/s2)
Re_p = 6.378e3;                         % Radius of Earth (km)
J2_p = 1.082e-3;                        % J2 parameter
I_p  = [1 1 1].';                       % Moment of inertia (kg*m2)
we_p = 7.292e-5;                    % Rotational speed of earth (rad/s)
%---

% Initial States
lat_p       = -33.90;                   % Lattitude (deg)
lon_p       = 18.41;                    % Longitude (deg)
alt_p       = 500;                      % Altitude (km)
rotx_p      = 0;                        % Satellite roll  Offset B/O (deg)
roty_p      = 0;                        % Satellite Pitch Offset B/O (deg)
rotz_p      = 0;                        % Satellite Yaw   Offset B/O (deg)
wx_p        = 2;                        % Satellite Roll  rate B/O (deg/s)
wy_p        = 0;                        % Satellite Pitch rate B/O (deg/s) 
wz_p        = 0;                        % Satellite Yaw rate B/O (deg/s)
inc_p       = -33.90;                   % Initial Inclination (deg)
%---

% Initialize orbit
[r_p, v_p, q_O2B_p, w_O2B_p] = ...
InitialiseOrbit( ...
lat_p, lon_p, alt_p, ...
rotx_p, roty_p, rotz_p, ...
wx_p, wy_p, wz_p, ...
Mu_p, we_p, 0, inc_p);
%---

% Set initial true state
x_true(:,1) = [r_p; v_p; q_O2B_p; w_O2B_p];
%---

%==========================================================================

%% Initialise Camera & Source Map =========================================
% Camera Parameters
imgWidth_cam        = 720;                  % Along track image width (pixels)
imgHeight_cam       = 720;                  % Cross-track image height (pixels)
focalLength_cam     = 0.58;                 % Focal length (m)
pixelSize_cam       = 17.4e-6;              % Pixel size (m)
sourceMapFile       = 'CapeStrip10.tif';     % The high-res GeoTIFF file
%---

%==========================================================================

%% Initialise Simulation & Dataset File ===================================

% Define the output file path
timestamp = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
filename = sprintf('Dataset_%s.mat', timestamp);
filepath = fullfile('C:\Users\Niel\Desktop\Masters\1. Satellite\4. Datasets', filename);
fprintf('Creating dataset file at: %s\n', filepath);

metadata = struct();
metadata.simulationTime = st;
metadata.sampleTime = dt_p;
metadata.physics.Mu = Mu_p;
metadata.physics.Re = Re_p;
metadata.physics.J2 = J2_p;
metadata.physics.I = I_p;
metadata.physics.we = we_p;
metadata.initial.lat  = lat_p;
metadata.initial.lon  = lon_p;
metadata.initial.alt  = alt_p;
metadata.initial.rotx = rotx_p;
metadata.initial.roty = roty_p;
metadata.initial.rotz = rotz_p;
metadata.initial.wx   = wx_p;
metadata.initial.wy   = wy_p;
metadata.initial.wz   = wz_p;
metadata.initial.inc  = inc_p;
metadata.camera.imgWidth = imgWidth_cam;
metadata.camera.imgHeight = imgHeight_cam;
metadata.camera.focalLength = focalLength_cam;
metadata.camera.pixelSize = pixelSize_cam;
metadata.info.dateCreated = datetime('now', 'TimeZone', 'Africa/Johannesburg');
metadata.info.description = 'Satellite pose estimation dataset generated with ray-tracing.';

% 1. Create the .mat file and save ONLY the metadata struct.
save(filepath, 'metadata', '-v7.3');

% 2. Create the matfile handle to modify the file.
m = matfile(filepath, 'Writable', true);

% 3. Pre-allocate the large arrays as TOP-LEVEL variables in the file.
m.poses = zeros(n_x, n_s);
m.images = zeros(imgHeight_cam, imgWidth_cam, 3, n_s, 'uint8');

% Write the initial pose
m.poses(:,1) = x_true(:,1);

% Write Time varaible
time = (0:n_s-1) * dt_p;
m.time = time;

% Progress bar setup
fig = uifigure('Name','Simulation Progress');
d = uiprogressdlg(fig, 'Title','Running Simulation & Generating Images', ...
    'Message','Initializing...', 'Indeterminate','off');
startTime = tic;
%==========================================================================

currentImage = GenerateSatelliteImage_RayTrace( ...
    x_true(1:3,1), ...      % r_I (Position)
    x_true(4:6,1), ...      % v_I (Velocity)
    x_true(7:10,1), ...     % q_O2B (Quaternion)
    imgWidth_cam, ...
    imgHeight_cam, ...
    focalLength_cam, ...
    pixelSize_cam, ...
    sourceMapFile, ...
    0, ...
    we_p);
m.images(:,:,:,1) = currentImage;
SaveSatelliteImages(currentImage,1);


%% Run Simulation & Save Directly to MAT-File =============================
for r = 2:n_s

    % Time ----------------------------------------------------------------
    t = (r-1)*dt_p;
    %----------------------------------------------------------------------

    % Plant Propagation (in local memory) ---------------------------------
    x_true(:,r) = Plant(x_true(:,r-1), dt_p, I_p, Mu_p, Re_p, J2_p);
    m.poses(:,r) = x_true(:,r);
    %----------------------------------------------------------------------

    % Image Generator -----------------------------------------------------
    currentImage = GenerateSatelliteImage_RayTrace( ...
        x_true(1:3,r), ...      % r_I (Position)
        x_true(4:6,r), ...      % v_I (Velocity)
        x_true(7:10,r), ...     % q_O2B (Quaternion)
        imgWidth_cam, ...
        imgHeight_cam, ...
        focalLength_cam, ...
        pixelSize_cam, ...
        sourceMapFile, ...
        t, ...
        we_p);
    
    m.images(:,:,:,r) = currentImage;
    SaveSatelliteImages(currentImage,r);
    % ----------------------------------------------------------------------

    % Progress bar --------------------------------------------------------
    elapsedTime = toc(startTime);
    progress = r / n_s;
    estTotalTime = elapsedTime / progress;
    estRemaining = estTotalTime - elapsedTime;
    d.Value = progress;
    d.Message = sprintf('Elapsed: %.2fs | %d%% | Est. remaining: %.2fs | Sample: %d | Footage Time: %.2fs | Time per sample: %.2fs', ...
        elapsedTime, round(progress*100), estRemaining,r+1,r*dt_p,elapsedTime/r);
    drawnow;  
    %----------------------------------------------------------------------
end

close(d);
close(fig);
fprintf('Simulation complete. Dataset successfully saved to %s\n', filename);
%==========================================================================