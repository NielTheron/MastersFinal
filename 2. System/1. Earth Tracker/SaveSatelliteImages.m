%======================================================================
% Niel Theron
% 05-09-2025
%=====================================================================

function SaveSatelliteImages(satellite_image,r)
    filename = sprintf('sat_image_%03d.png', r); 
    outputFolder = 'C:\Users\Niel\Desktop\1. Masters\2. System\1. Earth Tracker\SatelliteImages';
    fullpath = fullfile(outputFolder,filename);
    imwrite(satellite_image,fullpath);
end
