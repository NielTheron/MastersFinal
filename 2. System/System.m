%==========================================================================
% Simulator with ET Temporal Delay
% Modified by: Niel Theron
% Date: 12-06-2025
% 
% MODIFICATION: Added temporal delay to Earth Tracker (ET) measurements
%               z_ET measurements are now delayed by a specified number of timesteps
%==========================================================================
function System(app)
%% Simulation Parameters ==================================================

% Simulation Parameters
st      = evalin("base","st");
dt_p    = evalin("base","dt_p");
n_s     = round(st/dt_p);
enable_SaveImages = evalin("base","enable_SaveImages");
%---

% State Parameters
x_true  = evalin("base","x_true");
Mu_p    = evalin("base","Mu_p");
we_p    = evalin("base","we_p");
%---

% Camera Parameters
focalLength_cam   = evalin("base","focalLength_cam");
pixelSize_cam     = evalin("base","pixelSize_cam");
Ix                = evalin("base","Ix");
Iy                = evalin("base","Iy");
satelliteImage    = evalin("base","satelliteImage");
FD                = evalin("base","FD");
n_f               = evalin("base","n_f");
%---

% More Camera Parameters
alpha_m           = evalin("base","alpha_m");
alpha_ET          = evalin("base","alpha_ET");
k1                = evalin("base","k1");
k2                = evalin("base","k2");
k3                = evalin("base","k3");
p1                = evalin("base","p1");
p2                = evalin("base","p2");
R                 = evalin("base","R");
G                 = evalin("base","G");
B                 = evalin("base","B");
radial_enable     = evalin("base","radial_enable");
tangential_enable = evalin("base","tangential_enable");
chromatic_enable  = evalin("base","chromatic_enable");

radialDisPar      = [k1,k2,k3];
tangentialDisPar  = [p1,p2];
chromaticDisPar   = [R,G,B];
%---

% EKF Covariance Matrix Parameters
Prx = evalin("base","Prx"); Pry = evalin("base","Pry"); Prz = evalin("base","Prz");
Pvx = evalin("base","Pvx"); Pvy = evalin("base","Pvy"); Pvz = evalin("base","Pvz");
Pqs = evalin("base","Pqs"); Pqx = evalin("base","Pqx"); Pqy = evalin("base","Pqy"); Pqz = evalin("base","Pqz");
Pwx = evalin("base","Pwx"); Pwy = evalin("base","Pwy"); Pwz = evalin("base","Pwz");
%---

% EKF Process Noise Matrix Parameters
Qrx = evalin("base","Qrx"); Qry = evalin("base","Qry"); Qrz = evalin("base","Qrz");
Qvx = evalin("base","Qvx"); Qvy = evalin("base","Qvy"); Qvz = evalin("base","Qvz");
Qqs = evalin("base","Qqs"); Qqx = evalin("base","Qqx"); Qqy = evalin("base","Qqy"); Qqz = evalin("base","Qqz");
Qwx = evalin("base","Qwx"); Qwy = evalin("base","Qwy"); Qwz = evalin("base","Qwz");
%---

% Filter Initialisation Parameters
lat_f  = evalin("base","lat_f");
lon_f  = evalin("base","lon_f");
alt_f  = evalin("base","alt_f");
rotx_f = evalin("base","rotx_f");
roty_f = evalin("base","roty_f");
rotz_f = evalin("base","rotz_f");
wx_f   = evalin("base","wx_f");
wy_f   = evalin("base","wy_f");
wz_f   = evalin("base","wz_f");
Mu_f   = evalin("base","Mu_f");
we_f   = evalin("base","we_f");
dt_f   = evalin("base","dt_f");
inc_f  = evalin("base","inc_f");
I_f    = evalin("base","I_f");
Re_f   = evalin("base","Re_f");
J2_f   = evalin("base","J2_f");
%---

% Earth Tracker / Sensor Timing
dt_ET          = evalin("base","dt_ET");
focalLength_ET = evalin("base","focalLength_ET");
pixelSize_ET   = evalin("base","pixelSize_ET");
imgWidth_ET    = evalin("base","imgWidth_ET");
imgHeight_ET   = evalin("base","imgHeight_ET");
noise_ET       = evalin("base","noise_ET");
enable_ET      = evalin("base","enable_ET");
%---

% NEW: ET Temporal Delay Parameter
% Read the delay parameter from base workspace (number of ET sampling periods to delay)
if evalin("base","exist('delay_ET_periods','var')")
    delay_ET_periods = evalin("base","delay_ET_periods");
else
    delay_ET_periods = 0;  % Default: no delay
    fprintf('Warning: delay_ET_periods not found in workspace. Using default value of 0.\n');
end

% Convert ET periods to simulation timesteps
% delay_ET_periods is in units of dt_ET (ET sampling periods)
% We need to convert to units of dt_p (simulation timesteps)
delay_ET_steps = round(delay_ET_periods * dt_ET / dt_p);
%---

% GPS / GYR / ST / CSS / MAG enabling, noise & timing
dt_GPS    = evalin("base","dt_GPS");
dt_GYR    = evalin("base","dt_GYR");
dt_ST     = evalin("base","dt_ST");
dt_CSS    = evalin("base","dt_CSS");
dt_MAG    = evalin("base","dt_MAG");
%---

noise_GYR  = evalin("base","noise_GYR");
noise_GPS  = evalin("base","noise_GPS");
noise_ST   = evalin("base","noise_ST");
noise_CSS  = evalin("base","noise_CSS");
noise_MAG  = evalin("base","noise_MAG");
%---

enable_GPS = evalin("base","enable_GPS");
enable_GYR = evalin("base","enable_GYR");
enable_ST  = evalin("base","enable_ST");
enable_CSS = evalin("base","enable_CSS");
enable_MAG = evalin("base","enable_MAG");
%---

% GPS/GYR Drift Parameters
driftRate_GPS = evalin("base","driftRate_GPS");
driftRate_GYR = evalin("base","driftRate_GYR");
%---


%==========================================================================
%% Initialise EKF =========================================================

% EKF Variables
n_x = 13;
x_EKF = zeros(n_x,n_s);             % Initialise filter states
P_EKF = zeros(n_x,n_x,n_s);         % Initialise filter covariance matrix
%---

% Initialize EKF orbit
[r_f, v_f, q_O2B_f, w_O2B_f] = ...
    InitialiseOrbit( ...
    lat_f, lon_f, alt_f, ...
    rotx_f, roty_f, rotz_f, ...
    wx_f, wy_f, wz_f, ...
    Mu_f, we_f, 0, inc_f);
%---

% Initialize Filter
x_EKF(:,1) = [r_f; v_f; q_O2B_f; w_O2B_f];    % Estimated state
P_EKF(:,:,1) =  TuneP(Prx, Pry, Prz, Pvx, Pvy, Pvz, Pqs, Pqx, Pqy, Pqz, Pwx, Pwy, Pwz);     % Covariance matrix
Q_EKF =         TuneQ(Qrx, Qry, Qrz, Qvx, Qvy, Qvz, Qqs, Qqx, Qqy, Qqz, Qwx, Qwy, Qwz, dt_f);   % Process Noise Matrix
%---

%==========================================================================
%% Initialise Earth Tracker ===============================================

% Initialise Feature Detection
f_m = zeros(2,n_f,n_s);                 % Feature Pixel Locations for image (pixels)
f_d = zeros(2,n_f,n_s);                 % Feature Pixel Locations for distorted image (pixels)
%--

% Initialsie LineOfSight Varaibles
n_ET     = 3;                           % Number of measurements
R_ET     = (noise_ET/1000)^2*eye(3);         % Measurement Noise Covariance Matrix
z_ET     = zeros(n_ET,n_f,n_s);         % Earth tracker measurement (km) - CURRENT measurements
z_ET_delayed = zeros(n_ET,n_f,n_s);     % NEW: Delayed ET measurements for filter
y_ET     = zeros(n_ET,n_f,n_s);         % Estimated Earth tracker measurement (km)
K_ET     = zeros(n_x,n_ET,n_f,n_s);     % Earth tracker Kalman Gain (km)
H_ET     = zeros(n_ET,n_x,n_f,n_s);
%---

% Initialise Geolocation Varaibles
catalogue_geo   = zeros(3,n_f,n_s);     % Catalogue in lla
catalogue_eci   = zeros(3,n_f,n_s);     % Catalogue in ECI
catalogue_geo_delayed = zeros(3,n_f,n_s); % NEW: Delayed catalogue for filter
zhat_ET = zeros(3,n_f,n_s);
%---

%==========================================================================
%% Initialise Direct Sensors ==============================================

% Gyroscope
n_GYR       = 3;                                % Number of measurements
R_GYR       = deg2rad(noise_GYR)^2*eye(3);      % Gyroscope sensor noise matrix
z_GYR       = zeros(n_GYR,n_s);                 % Gyroscope measurement
zhat_GYR    = zeros(n_GYR,n_s);                 % Gyroscope estimated measurement
y_GYR       = zeros(n_GYR,n_s);                 % Gyroscope innovation
K_GYR       = zeros(n_x,n_GYR,n_s);             % Gyroscope Kalman Gain
H_GYR       = zeros(n_GYR,n_x,n_s);
drift_GYR   = zeros(n_GYR,1);                   % Gyroscope drift buffer
%---

% GPS
n_GPS       = 3;                                % Number of measurements
R_GPS       = diag([(noise_GPS/111000)^2;(noise_GPS/111000)^2;(noise_GPS/1000)^2]);
z_GPS       = zeros(n_GPS,n_s);                 % GPS measurement
zhat_GPS    = zeros(n_GPS,n_s);                 % GPS estimated measurement
y_GPS       = zeros(n_GPS,n_s);                 % GPS innovation
K_GPS       = zeros(n_x,n_GPS,n_s);             % GPS Kalman Gain
H_GPS       = zeros(n_GPS,n_x,n_s);
drift_GPS   = zeros(n_GPS,1);                   % GPS drift buffer
%----

% Star Tracker
n_ST        = 3;                                % Number of measurements
R_ST        = deg2rad(noise_ST/3600)^2 * eye(3);    % Star tracker noise matrix (rad)
z_ST        = zeros(n_ST,20,n_s);                  % Star tracker measurements
zhat_ST     = zeros(n_ST,20,n_s);                  % Star tracker estimated measurements
y_ST        = zeros(n_ST,20,n_s);                  % Star tracker innovation
H_ST        = zeros(n_ST,n_x,20,n_s);
K_ST        = zeros(n_x,n_ST,20,n_s);              % Star tracker Kalman Gain
%---

% Magnetometer
n_MAG       = 3;                                % Number of measurements
R_MAG       = noise_MAG^2*eye(n_MAG);           % Magnetometer noise matrix
z_MAG       = zeros(n_MAG,n_s);                 % Magnetometer measurements
zhat_MAG    = zeros(n_MAG,n_s);                 % Magnetometer estimated measurements
y_MAG       = zeros(n_MAG,n_s);                 % Magnetometer innovation
H_MAG       = zeros(n_MAG,n_x,n_s);
K_MAG       = zeros(n_x,n_MAG,n_s);             % Magnetimeter Kalman Gain
%---

% Coarse Sun Sensor
n_CSS       = 3;                                % Number of measurements
CSS_angular_uncertainty = noise_CSS * 1.2;  % ~6° (conservative estimate)
R_CSS       = deg2rad(CSS_angular_uncertainty)^2 * eye(3);
z_CSS       = zeros(n_CSS,n_s);                 % Coarse sun sensor measurement
zhat_CSS    = zeros(n_CSS,n_s);                 % Coarse sun sensor estimated measurement
y_CSS       = zeros(n_CSS,n_s);                 % Coarse sun sensor innovation
K_CSS       = zeros(n_x,n_CSS,n_s);             % Coarse sun sensor Kalman Gain
H_CSS       = zeros(n_CSS,n_x,n_s);
%---

%==========================================================================
%% Initialise Simulation ==================================================

% Progress bar
fig = uifigure('Name','Simulation Progress');
d = uiprogressdlg(fig, 'Title','Running Simulation', ...
    'Message','Initializing...', 'Indeterminate','off');
startTime = tic;
%---

% Display delay information
if delay_ET_periods > 0
    fprintf('\n╔════════════════════════════════════════╗\n');
    fprintf('║   ET TEMPORAL DELAY ENABLED            ║\n');
    fprintf('╚════════════════════════════════════════╝\n');
    fprintf('  Delay: %d ET sampling period(s)\n', delay_ET_periods);
    fprintf('  ET sampling rate: %.3f Hz (dt_ET = %.3f s)\n', 1/dt_ET, dt_ET);
    fprintf('  Actual delay: %.3f seconds\n', delay_ET_periods * dt_ET);
    fprintf('  Equivalent to %d simulation timesteps (dt_p = %.3f s)\n', delay_ET_steps, dt_p);
    fprintf('  Filter will use measurements from t-%.3fs\n\n', delay_ET_periods * dt_ET);
end
%---

%==========================================================================
%% Run Simulation =========================================================
for r = 2:n_s
    drawnow; 
    % Stop simulation -----------------------------------------------------
    if ~app.SimulationRunning
        fprintf('Simulation stopped by user at sample %d\n', r);
        break;
    end
    %----------------------------------------------------------------------

    % Variables -----------------------------------------------------------
    u = r - 1;
    t = u*dt_p;

    %----------------------------------------------------------------------

    % Earth Tracker Measurement -------------------------------------------
    if mod(t,dt_ET) == 0 && enable_ET
     
        % Apply Lens Distortion -------------------------------------------
        distortedImage = LensDistortion(satelliteImage(:,:,:,r), ...
                radialDisPar, tangentialDisPar, chromaticDisPar, ...
                radial_enable, tangential_enable, chromatic_enable);
        %-------------------------------------------------------------------

        % Feature Detection ---------------------------------------------------
        [f_m(:,:,r), grayImage] = FeatureDetection(distortedImage, n_f,FD);
        [f_d(:,:,r), ~]         = FeatureDetection(satelliteImage(:,:,:,r), n_f,FD);
        %----------------------------------------------------------------------
        
        % Generate Catalogue ----------------------------------------
        catalogue_geo(:,:,r) = Catalogue(f_d(:,:,r), x_true(:,r), ...
            focalLength_cam, pixelSize_cam, alpha_m, Ix, Iy, we_p, t);

        catalogue_eci(:,:,r) = LLA2ECI(catalogue_geo(:,:,r), we_p, t);
        %------------------------------------------------------------

        % Save Images ----------------------------------------------
        if enable_SaveImages
            SaveSatelliteImages(satelliteImage(:,:,:,r),r);
            SaveDistortedImages(distortedImage,r);
            SaveFeatureImages(grayImage, f_m(:,:,r),r);
        end
        %=-----------------------------------------------------------

        
        % Create Sensor Measurements (CURRENT, not delayed yet)
        z_ET(:,:,r) = EarthTracker(f_m(:,:,r),imgWidth_ET,imgHeight_ET,focalLength_ET, pixelSize_ET,alpha_ET);
    else
        z_ET(:,:,r) = zeros(n_ET,n_f);
    end
    %----------------------------------------------------------------------
    
    % NEW: Apply Temporal Delay to ET Measurements ------------------------
    % The filter uses measurements from (r - delay_ET_steps) timesteps ago
    if r > delay_ET_steps
        % Use delayed measurement from previous timestep
        delayed_index = r - delay_ET_steps;
        z_ET_delayed(:,:,r) = z_ET(:,:,delayed_index);
        catalogue_geo_delayed(:,:,r) = catalogue_geo(:,:,delayed_index);
    else
        % During initial timesteps when buffer isn't full, use zeros or
        % you could use current measurements (no delay)
        z_ET_delayed(:,:,r) = zeros(n_ET,n_f);  % Option 1: Use zeros
        % z_ET_delayed(:,:,r) = z_ET(:,:,r);    % Option 2: Use current (no delay yet)
        catalogue_geo_delayed(:,:,r) = zeros(3,n_f);  % Corresponding catalogue
    end
    %----------------------------------------------------------------------

    % Sensors -----------------------------------------------------------
    [z_GPS(:,r), z_GYR(:,r), z_ST(:,:,r), z_CSS(:,r), z_MAG(:,r), drift_GPS, drift_GYR] = ...
        SampleSensors(x_true(:,r), t, we_p, Mu_p, dt_p,                ...
        dt_GPS, noise_GPS, enable_GPS, drift_GPS, driftRate_GPS,  ...
        dt_GYR, noise_GYR, enable_GYR, drift_GYR, driftRate_GYR,  ...
        dt_ST,  noise_ST,  enable_ST,                             ...
        dt_CSS, noise_CSS, enable_CSS,                            ...
        dt_MAG, noise_MAG, enable_MAG);
    % --------------------------------------------------------------------

    % EKF ---------------------------------------------------------------
    
    % Position
    % x_EKF(:,r-1) = [x_EKF(1:6,r-1);x_true(7:13,r-1)]; 
    % P_EKF(:,:, r-1) = [P_EKF(6:6,6:6,r-1), zeros(6,7); zeros(7,6); zeros(7,7), ];  

    % Attitude
    % x_EKF(:,r-1) = [x_true(1:6,r-1);x_EKF(7:13,r-1)]; 
    % P_EKF(:,:, r-1) = [zeros(6,6), zeros(6,7); zeros(7,6), P_EKF(7:13,7:13,r-1)];

    j = mod(t,dt_f);
    if j == 0
        % MODIFIED: Pass delayed measurements to the EKF
        [x_EKF(:,r),              ...
         P_EKF(:,:,r),            ...
         zhat_ET(:,:,r), K_ET(:,:,:,r), H_ET(:,:,:,r), ...
         zhat_ST(:,:,r), K_ST(:,:,:,r), H_ST(:,:,:,r), ...
         zhat_CSS(:,r),  K_CSS(:,:,r),  H_CSS(:,:,r), ...
         zhat_MAG(:,r),  K_MAG(:,:,r),  H_MAG(:,:,r), ...
         zhat_GYR(:,r),  K_GYR(:,:,r),  H_GYR(:,:,r), ...
         zhat_GPS(:,r),  K_GPS(:,:,r),  H_GPS(:,:,r)   ] = EKF(      ...
         catalogue_geo_delayed(:,:,r), ... % Use delayed catalogue
         x_EKF(:,r-1),                ...
         P_EKF(:,:,r-1),              ...
         I_f,Q_EKF,dt_p,Mu_f,Re_f,J2_f,we_f,t, ...
         z_ET_delayed(:,:,r), z_ST(:,:,r), z_GPS(:,r), z_GYR(:,r), z_MAG(:,r), z_CSS(:,r), ... % Use delayed z_ET
         R_ET, R_ST, R_GPS, R_GYR, R_MAG, R_CSS);
    else
        x_EKF(:,r) =  x_EKF(:,r-1);
        P_EKF(:,:,r) =  P_EKF(:,:,r-1);
    end
    %----------------------------------------------------------------------

    % Calculating Errors --------------------------------------------------
    % MODIFIED: Calculate innovation using delayed measurements
    y_ET(:,:,r) = z_ET_delayed(:,:,r) - zhat_ET(:,:,r);
    y_ST(:,:,r) = z_ST(:,:,r) - zhat_ST(:,:,r);
    y_GPS(:,r) = z_GPS(:,r) - zhat_GPS(:,r);
    y_GYR(:,r) = z_GYR(:,r) - zhat_GYR(:,r);
    y_CSS(:,r) = z_CSS(:,r) - zhat_CSS(:,r);
    y_MAG(:,r) = z_MAG(:,r) - zhat_MAG(:,r);
    %-----------------------------------------------------------

    % Estimated Video -----------------------------------------------------
    if mod(t,dt_ET) == 0 && enable_ET
        if enable_SaveImages
            SaveEstimatedImage(grayImage,z_ET(:,:,r),zhat_ET(:,:,r),Ix,Iy,pixelSize_cam,focalLength_cam,r);
        end
    end
    %----------------------------------------------------------------------

    % Progress bar --------------------------------------------------------
    elapsedTime = toc(startTime);
    progress = r / n_s;
    estTotalTime = elapsedTime / progress;
    estRemaining = estTotalTime - elapsedTime;
    d.Value = progress;
    d.Message = sprintf('Elapsed: %.2fs | %d%% | Est. remaining: %.2fs | Sample: %d | Footage Time: %.2fs | Time per sample: %.2fs', ...
        elapsedTime, round(progress*100), estRemaining,r,t*dt_ET,elapsedTime/r);
    drawnow;  
    %----------------------------------------------------------------------

    % Stop Simulation (final check) --------------------------------------
    if ~app.SimulationRunning
        fprintf('Simulation stopped by user at sample %d\n', r+1);
        break;
    end
    %----------------------------------------------------------------------
end

%==========================================================================
%% Save Variables (whether completed or stopped) =========================

% Check if stopped BEFORE modifying anything
wasStopped = ~app.SimulationRunning;

% Trim arrays to actual size (remove unused pre-allocated space)
actual_samples = r;  % +1 because we have initial condition
x_EKF           = x_EKF(:, 1:actual_samples);
P_EKF           = P_EKF(:, :, 1:actual_samples);
z_ET            = z_ET(:, :, 1:actual_samples);
z_ET_delayed    = z_ET_delayed(:, :, 1:actual_samples);  % NEW: Save delayed measurements
z_GPS           = z_GPS(:, 1:actual_samples);
z_GYR           = z_GYR(:, 1:actual_samples);
z_ST            = z_ST(:, :, 1:actual_samples);
z_CSS           = z_CSS(:, 1:actual_samples);
z_MAG           = z_MAG(:, 1:actual_samples);
zhat_ET         = zhat_ET(:, :, 1:actual_samples);
zhat_ST         = zhat_ST(:, :, 1:actual_samples);
zhat_MAG        = zhat_MAG(:, 1:actual_samples);
zhat_CSS        = zhat_CSS(:, 1:actual_samples);
zhat_GPS        = zhat_GPS(:, 1:actual_samples);
zhat_GYR        = zhat_GYR(:, 1:actual_samples);
y_ET            = y_ET(:, :, 1:actual_samples);
y_GPS           = y_GPS(:, 1:actual_samples);
y_GYR           = y_GYR(:, 1:actual_samples);
y_ST            = y_ST(:, :, 1:actual_samples);
y_CSS           = y_CSS(:, 1:actual_samples);
y_MAG           = y_MAG(:, 1:actual_samples);
catalogue_eci   = catalogue_eci(:, :, 1:actual_samples);
catalogue_geo   = catalogue_geo(:, :, 1:actual_samples);
catalogue_geo_delayed = catalogue_geo_delayed(:, :, 1:actual_samples); % NEW
f_m             = f_m(:, :, 1:actual_samples);
f_d             = f_d(:, :, 1:actual_samples);
K_ET            = K_ET(:, :, :, 1:actual_samples);
K_GPS           = K_GPS(:, :, 1:actual_samples);
K_GYR           = K_GYR(:, :, 1:actual_samples);
K_ST            = K_ST(:, :, :, 1:actual_samples);
K_MAG           = K_MAG(:, :, 1:actual_samples);
K_CSS           = K_CSS(:, :, 1:actual_samples);
H_ET            = H_ET(:, :, :, 1:actual_samples);
H_GPS           = H_GPS(:, :, 1:actual_samples);
H_GYR           = H_GYR(:, :, 1:actual_samples);
H_ST            = H_ST(:, :, :, 1:actual_samples);
H_MAG           = H_MAG(:, :, 1:actual_samples);
H_CSS           = H_CSS(:, :, 1:actual_samples);
%---

%% Update progress dialog and show final status
if wasStopped
    % Simulation was stopped by user
    d.Value = r / (n_s-1);
    d.Message = sprintf('Stopped - Saving data... %d/%d samples', actual_samples, n_s);
    drawnow;
    
    % Console output
    fprintf('\n╔════════════════════════════════════════╗\n');
    fprintf('║   SIMULATION STOPPED BY USER          ║\n');
    fprintf('╚════════════════════════════════════════╝\n');
    fprintf('  Samples completed: %d/%d\n', actual_samples, n_s);
    fprintf('  Elapsed time: %.2fs\n', toc(startTime));
    fprintf('  Data saved successfully.\n\n');
    
else
    % Simulation completed normally
    d.Value = 1;
    d.Message = sprintf('Complete - Saving results... %d samples', actual_samples);
    drawnow;
    
    % Console output
    fprintf('\n╔════════════════════════════════════════╗\n');
    fprintf('║   SIMULATION COMPLETE                  ║\n');
    fprintf('╚════════════════════════════════════════╝\n');
    fprintf('  Total samples: %d\n', actual_samples);
    fprintf('  Total time: %.2fs\n', toc(startTime));
    fprintf('  Avg time/sample: %.3fs\n', toc(startTime)/actual_samples);
    fprintf('  Footage time: %.2fs\n\n', (actual_samples-1)*dt_ET);
end
%---

%% Save Variables to Base Workspace ======================================

% Save main simulation results
assignin('base', 'x_EKF', x_EKF);
assignin('base', 'P_EKF', P_EKF);
assignin('base', 'Q_EKF',Q_EKF);
assignin('base', 'actual_samples', actual_samples);
assignin('base', 'n_s', n_s);
assignin('base', 'delay_ET_periods', delay_ET_periods);  % NEW: Save delay in ET periods
assignin('base', 'delay_ET_steps', delay_ET_steps);      % NEW: Save delay in timesteps
%---

% Save measurement data
assignin('base', 'z_ET', z_ET);                    % Current measurements
assignin('base', 'z_ET_delayed', z_ET_delayed);    % NEW: Delayed measurements
assignin('base', 'z_GPS', z_GPS);
assignin('base', 'z_GYR', z_GYR);
assignin('base', 'z_ST', z_ST);
assignin('base', 'z_CSS', z_CSS);
assignin('base', 'z_MAG', z_MAG);
assignin('base', 'zhat_ET', zhat_ET);
assignin('base', 'zhat_ST', zhat_ST);
assignin('base', 'zhat_MAG', zhat_MAG);
assignin('base', 'zhat_CSS', zhat_CSS);
assignin('base', 'zhat_GPS', zhat_GPS);
assignin('base', 'zhat_GYR', zhat_GYR);
assignin('base', 'y_ET', y_ET);
assignin('base', 'y_GPS', y_GPS);
assignin('base', 'y_GYR', y_GYR);
assignin('base', 'y_ST', y_ST);
assignin('base', 'y_CSS', y_CSS);
assignin('base', 'y_MAG', y_MAG);
%---

% Save other useful variables
assignin('base', 'catalogue_eci', catalogue_eci);
assignin('base', 'catalogue_geo', catalogue_geo);
assignin('base', 'catalogue_geo_delayed', catalogue_geo_delayed);  % NEW
assignin('base', 'f_m', f_m);
assignin('base', 'f_d', f_d);
%---

% Save Kalman gain matrices
assignin('base', 'K_ET', K_ET);
assignin('base', 'K_GPS', K_GPS);
assignin('base', 'K_GYR', K_GYR);
assignin('base', 'K_ST', K_ST);
assignin('base', 'K_MAG', K_MAG);
assignin('base', 'K_CSS', K_CSS);
%---

% Save Measurement matrices
assignin('base', 'H_ET', H_ET);
assignin('base', 'H_GPS', H_GPS);
assignin('base', 'H_GYR', H_GYR);
assignin('base', 'H_ST', H_ST);
assignin('base', 'H_MAG', H_MAG);
assignin('base', 'H_CSS', H_CSS);
%---


% Save noise matrices
assignin('base', 'R_ET', R_ET);
assignin('base', 'R_GPS', R_GPS);
assignin('base', 'R_GYR', R_GYR);
assignin('base', 'R_ST', R_ST);
assignin('base', 'R_CSS', R_CSS);
assignin('base', 'R_MAG', R_MAG);
%---

fprintf('Simulation variables saved to base workspace.\n');
if delay_ET_periods > 0
    fprintf('Note: Both z_ET (current) and z_ET_delayed (used by filter) have been saved.\n');
    fprintf('      Delay = %d ET period(s) = %.3f seconds = %d timesteps\n', ...
            delay_ET_periods, delay_ET_periods * dt_ET, delay_ET_steps);
end

%% Clean up and show completion dialog
% Update final message on progress dialog
if wasStopped
    d.Message = sprintf('✓ Stopped - Data saved (%d/%d samples)', actual_samples, n_s);
else
    d.Message = sprintf('✓ Complete - All data saved (%d samples)', actual_samples);
end
drawnow;

% Brief pause to show final message
pause(1.5);

% Close progress dialog
close(d);
close(fig);

%% Update app UI state
if isvalid(app)
    % Reset app state
    app.RunSimulationButton.Enable = 'on';
    app.StopSimulationButton.Enable = 'off';
    app.SimulationRunning = false;
    
    % Show completion alert based on what actually happened
    if wasStopped
        uialert(app.UIFigure, ...
            sprintf('Simulation stopped by user.\n\n%d/%d samples completed\nElapsed time: %.2fs\n\nAll data has been saved.', ...
                    actual_samples, n_s, toc(startTime)), ...
            'Simulation Stopped', ...
            'Icon', 'warning');
    else  % Completed normally
        uialert(app.UIFigure, ...
            sprintf('Simulation completed successfully!\n\n%d samples processed\nTotal time: %.2fs\nAverage time per sample: %.3fs\n\nAll data has been saved.', ...
                    actual_samples, toc(startTime), toc(startTime)/actual_samples), ...
            'Simulation Complete', ...
            'Icon', 'success');
    end
end

end

