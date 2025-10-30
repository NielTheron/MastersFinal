%==========================================================================
% Niel Theron
% 10-06-2025
%==========================================================================
% The purpose of this function is to load all the other functions
%==========================================================================

function LoadPath()

addpath("1. Earth Tracker\")
addpath("1. Earth Tracker\FeatureImages\")
addpath("1. Earth Tracker\SatelliteImages\")
addpath("1. Earth Tracker\DistortedImages\")
addpath("1. Earth Tracker\EstimatedImages\")

addpath("2. Sensors\")

addpath("3. EKF\")
addpath("3. EKF\Prediction\")
addpath("3. EKF\Tune\")
addpath("3. EKF\Update\")
addpath("3. EKF\Update\CSS\")
addpath("3. EKF\Update\ET\")
addpath("3. EKF\Update\GPS\")
addpath("3. EKF\Update\Gyro\")
addpath("3. EKF\Update\Mag\")
addpath("3. EKF\Update\ST\")

addpath("4. Transformations\")
addpath("4. Transformations\Convertions\")
addpath("4. Transformations\Transformations\")
addpath("4. Transformations\Rotations\")

addpath("5. Admin\")

addpath("6. Plots\")
addpath("6. Plots\Plot Catalogues\")
addpath("6. Plots\Plot Earth Tracker\")
addpath("6. Plots\Plot Sensors\")
addpath("6. Plots\Plot Sensors\CSS\")
addpath("6. Plots\Plot Sensors\GPS\")
addpath("6. Plots\Plot Sensors\MAG\")
addpath("6. Plots\Plot Sensors\GYR\")
addpath("6. Plots\Plot Sensors\ST\")
addpath("6. Plots\Plot Images\")
addpath("6. Plots\Plot System States\")
addpath("6. Plots\Plot Visual\")
addpath("6. Plots\Plot Seperate\")

addpath("7. Datasets\")
end

%==========================================================================