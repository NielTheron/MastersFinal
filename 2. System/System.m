%==========================================================================
% Simulator
% Niel Theron
% 12-06-2025
%==========================================================================

%% Simulation Parameters ==================================================

% Simulation Parameters
st      = evalin("base","st");
dt_p    = evalin("base","dt_p");
n_s     = round(st/dt_p);
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
    Mu_f, we_f, dt_f, inc_f);
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
R_ET     = noise_ET^2*eye(3);         % Measurement Noise Covariance Matrix
z_ET     = zeros(n_ET,n_f,n_s);         % Earth tracker measurement (km)
y_ET     = zeros(n_ET,n_f,n_s);         % Estimated Earth tracker measurement (km)
K_ET     = zeros(n_x,n_ET,n_f,n_s);     % Earth tracker Kalman Gain (km)
%---

% Initialise Geolocation Varaibles
catalogue_geo   = zeros(3,n_f,n_s);     % Catalogue in lla
catalogue_eci   = zeros(3,n_f,n_s);     % Catalogue in ECI
zhat_ET = zeros(3,n_f,n_s);
%---

%==========================================================================
%% Initialise Direct Sensors ==============================================

% Gyroscope
n_GYR       = 3;                                % Number of measurements
R_GYR       = deg2rad(noise_GYR)^2*eye(3);      % Gyroscope sensor noise matrix
z_GYR       = zeros(n_GYR,n_s);                 % Gyroscope measurement
y_GYR       = zeros(n_GYR,n_s);                 % Gyroscope estimated measurement
K_GYR       = zeros(n_x,n_GYR,n_s);             % Gyroscope Kalman Gain
drift_GYR   = zeros(n_GYR,1);                   % Gyroscope drift buffer
%---

% GPS
n_GPS       = 3;                                % Number of measurements
R_GPS = diag([(noise_GPS/111)^2;(noise_GPS/111)^2;(noise_GPS/1000)^2]);
z_GPS       = zeros(n_GPS,n_s);                 % GPS measurement
y_GPS       = zeros(n_GPS,n_s);                 % GPS estimated measurement
K_GPS       = zeros(n_x,n_GPS,n_s);             % GPS Kalman Gain
drift_GPS   = zeros(n_GPS,1);                   % GPS drift buffer
%----

% Star Tracker
n_ST        = 3;                                % Number of measurements
R_ST        = deg2rad(noise_ST)^2*eye(n_ST);    % Star tracker noise matrix (rad)
z_ST        = zeros(n_ST,20,n_s);                  % Star tracker measurements
y_ST        = zeros(n_ST,n_s);                  % Star tracker estimated measurements
K_ST        = zeros(n_x,n_ST,n_s);              % Star tracker Kalman Gain
%---

% Magnetometer
n_MAG       = 3;                                % Number of measurements
R_MAG       = noise_MAG^2*eye(n_MAG);           % Magnetometer noise matrix (rad)
z_MAG       = zeros(n_MAG,n_s);                 % Magnetometer measurements
y_MAG       = zeros(n_MAG,n_s);                 % Magnetometer estimated measurements
K_MAG       = zeros(n_x,n_MAG,n_s);             % Magnetimeter Kalman Gain
%---

% Coarse Sun Sensor
n_CSS       = 3;                                % Number of measurements
R_CSS       = noise_CSS^2*eye(n_CSS);           % Coarse sun sensor noise matrix (rad)
z_CSS       = zeros(n_CSS,n_s);                 % Coarse sun sensor measurement
y_CSS       = zeros(n_CSS,n_s);                 % Coarse sun sensor estimated measurement
K_CSS       = zeros(n_x,n_CSS,n_s);             % Coarse sun sensor Kalman Gain
%---

%==========================================================================
%% Initialise Simulation ==================================================

% Progress bar
fig = uifigure('Name','Simulation Progress');
d = uiprogressdlg(fig, 'Title','Running Simulation', ...
    'Message','Initializing...', 'Indeterminate','off');
startTime = tic;
%---

% Global Stop
global STOP_SIMULATION;
STOP_SIMULATION = false;
%---


%==========================================================================
%% Run Simulation =========================================================
for r = 1:n_s-1
    % Stop simulation -----------------------------------------------------
    if STOP_SIMULATION
        fprintf('Simulation stopped by user at sample %d\n', r);
        break;
    end
    %----------------------------------------------------------------------

    % Variables -----------------------------------------------------------
    t = r*dt_p;
    %----------------------------------------------------------------------

    % Earth Tracker Measurement -------------------------------------------
    if mod(t,dt_ET) == 0 && enable_ET

        % Apply Lens Distortion -------------------------------------------
        distortedImage(:,:,:,r) = LensDistortion(satelliteImage(:,:,:,r), ...
                radialDisPar, tangentialDisPar, chromaticDisPar, ...
                radial_enable, tangential_enable, chromatic_enable);
        SaveSatelliteImages(satelliteImage(:,:,:,r),r);
        SaveDistortedImages(distortedImage(:,:,:,r),r);
        %-------------------------------------------------------------------

        % Feature Detection ---------------------------------------------------
        [f_m(:,:,r), grayImage] = FeatureDetection(distortedImage(:,:,:,r), n_f,FD);
        [f_d(:,:,r), ~]         = FeatureDetection(satelliteImage(:,:,:,r), n_f,FD);
        SaveFeatureImages(grayImage, f_m(:,:,r),r);
        %----------------------------------------------------------------------
        
        % Generate Catalogue ----------------------------------------
        [catalogue_geo(:,:,r), catalogue_eci(:,:,r)] = Catalogue(f_d(:,:,r), x_true(:,r), ...
            focalLength_cam, pixelSize_cam, alpha_m, Ix, Iy, we_p, t);
        %------------------------------------------------------------
        
        % Create Sensor Measurements 
        z_ET(:,:,r) = EarthTracker(f_m(:,:,r),imgWidth_ET,imgHeight_ET,focalLength_ET, pixelSize_ET,alpha_ET, noise_ET);
    else
        z_ET(:,:,r) = zeros(n_ET,n_f);
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
    j = mod(t,dt_f);
    if j == 0
        [zhat_ET(:,:,r), x_EKF(:,r+1), P_EKF(:,:,r+1), K_ET(:,:,:,r)] = EKF( ...
            catalogue_geo(:,:,r),x_EKF(:,r),P_EKF(:,:,r),I_f,Q_EKF,dt_p,Mu_f,Re_f,J2_f,we_f,t, ...
            z_ET(:,:,r), z_ST(:,:,r), z_GPS(:,r), z_GYR(:,r), z_MAG(:,r), z_CSS(:,r), ...
            R_ET, R_ST, R_GPS, R_GYR, R_MAG, R_CSS);
    else
        x_EKF(:,r+1) =  x_EKF(:,r);
        P_EKF(:,:,r+1) =  P_EKF(:,:,r);
    end
    %----------------------------------------------------------------------

    % Estimated Video -----------------------------------------------------
    if mod(t,dt_ET) == 0 && enable_ET
        SaveEstimatedImage(grayImage,z_ET(:,:,r),zhat_ET(:,:,r),Ix,Iy,pixelSize_cam,focalLength_cam,r)
    end
    %----------------------------------------------------------------------

    % Progress bar --------------------------------------------------------
    elapsedTime = toc(startTime);
    progress = r / (n_s-1);
    estTotalTime = elapsedTime / progress;
    estRemaining = estTotalTime - elapsedTime;
    d.Value = progress;
    d.Message = sprintf('Elapsed: %.2fs | %d%% | Est. remaining: %.2fs | Sample: %d | Footage Time: %.2fs | Time per sample: %.2fs', ...
        elapsedTime, round(progress*100), estRemaining,r+1,r*dt_ET,elapsedTime/r);
    %----------------------------------------------------------------------


    % Stop Simulation -----------------------------------------------------
    if STOP_SIMULATION
        fprintf('Simulation stopped by user at sample %d\n', r+1);
        break;
    end
    %----------------------------------------------------------------------
end

%==========================================================================
%% Save Variables (whether completed or stopped) =========================

% Trim arrays to actual size (remove unused pre-allocated space)
actual_samples = r + 1;  % +1 because we have initial condition
x_EKF           = x_EKF(:, 1:actual_samples);
P_EKF           = P_EKF(:, :, 1:actual_samples);
z_ET            = z_ET(:, :, 1:actual_samples);
z_GPS           = z_GPS(:, 1:actual_samples);
z_GYR           = z_GYR(:, 1:actual_samples);
z_ST            = z_ST(:,:, 1:actual_samples);
z_CSS           = z_CSS(:, 1:actual_samples);
z_MAG           = z_MAG(:, 1:actual_samples);
catalogue_eci   = catalogue_eci(:, :, 1:actual_samples);
catalogue_geo   = catalogue_geo(:, :, 1:actual_samples);
f_m             = f_m(:, :, 1:actual_samples);
f_d             = f_d(:, :, 1:actual_samples);
K_ET            = K_ET(:, :, :, 1:actual_samples);
%---

%% Update progress dialog based on completion status
if STOP_SIMULATION
    d.Value = r / (n_s-1);
    d.Message = sprintf('✓ STOPPED BY USER ✓ | Elapsed: %.2fs | %d/%d samples completed | Data saved', ...
        toc(startTime), actual_samples, n_s);
    d.Title = 'Simulation Stopped - Data Saved - Click X to close';
    fprintf('Simulation stopped by user. Data saved (%d samples)\n', actual_samples);
else
    d.Value = 1;
    d.Message = sprintf('✓ SIMULATION COMPLETE ✓ | Total: %.2fs | %d samples processed | Final footage time: %.2fs | Avg time per sample: %.2fs', ...
        toc(startTime), actual_samples, (actual_samples-1)*dt_ET, toc(startTime)/actual_samples);
    d.Title = 'Simulation Complete - Click X to close';
    fprintf('Simulation completed successfully! (%d samples)\n', actual_samples);
end
%---

% Keep references to prevent garbage collection
assignin('base', 'progress_dialog', d);
assignin('base', 'progress_figure', fig);
%---

% Reset stop flag
STOP_SIMULATION = false;
%---

%% Clean Processes ========================================================
delete('temp_image.png')
%==========================================================================

%% Save Variables to Base Workspace ======================================

% Save main simulation results
assignin('base', 'x_EKF', x_EKF);
assignin('base', 'P_EKF', P_EKF);
assignin('base', 'Q_EKF',Q_EKF);
assignin('base', 'actual_samples', actual_samples);
assignin('base', 'n_s', actual_samples);
%---

% Save measurement data
assignin('base', 'z_ET', z_ET);
assignin('base', 'z_GPS', z_GPS);
assignin('base', 'z_GYR', z_GYR);
assignin('base', 'z_ST', z_ST);
assignin('base', 'z_CSS', z_CSS);
assignin('base', 'z_MAG', z_MAG);
assignin('base', 'zhat_ET', zhat_ET);
%---

% Save other useful variables
assignin('base', 'catalogue_eci', catalogue_eci);
assignin('base', 'catalogue_geo', catalogue_geo);
assignin('base', 'f_m', f_m);
assignin('base', 'f_d', f_d);
%---

% Save Kalman gain matrices
assignin('base', 'K_ET', K_ET);
assignin('base', 'K_GPS', K_GPS);
assignin('base', 'K_GYR', K_GYR);
assignin('base', 'K_ST', K_ST);
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