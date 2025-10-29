%==========================================================================
% Niel Theron
% 04-09-2025
%==========================================================================

function SaveFeatureImages(grayImage,locations,r)
    rgbImage = repmat(grayImage, [1, 1, 3]);
    radii = 10;
    locations = locations';
    numPoints = size(locations, 1);
    circles = [locations, repmat(radii, numPoints, 1)];
    feature_image = insertShape(rgbImage, 'FilledCircle', circles,'Color', 'red', 'Opacity', 1);
    filename = sprintf('feature_image_%03d.png', r);  % Zero-padded filename like sat_image_001.mat
    outputFolder = 'C:\Users\Niel\Desktop\1. Masters\2. System\1. Earth Tracker\FeatureImages';
    fullpath = fullfile(outputFolder,filename);
    imwrite(feature_image,fullpath);
end

