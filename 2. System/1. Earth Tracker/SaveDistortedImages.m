%======================================================================
% Niel Theron
% 05-09-2025
%=====================================================================

function SaveDistortedImages(distorted_image,r)
    filename = sprintf('distorted_image_%03d.png', r); 
    outputFolder = 'C:\Users\Niel\Desktop\Masters\2. System\1. Earth Tracker\DistortedImages';
    fullpath = fullfile(outputFolder,filename);
    imwrite(distorted_image,fullpath);
end
