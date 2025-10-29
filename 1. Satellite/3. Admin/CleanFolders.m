%==========================================================================
% 10-06-2025
% Niel Theron
%==========================================================================
% The purpose of this function is to clear all the files in the image
% folders to make sure all images used in the analysis is the newest data
%==========================================================================

function CleanFolders()

    % Clear satellite Images
    folderPath = 'C:\Users\Niel\Desktop\1. Masters\1. Satellite\2. Image Generator\SatelliteImages'; 
    if exist(folderPath, 'dir')
        rmdir(folderPath, 's'); % 's' option removes the directory and all its contents
    end
    mkdir(folderPath);
    addpath(folderPath)
    %---

end

%==========================================================================