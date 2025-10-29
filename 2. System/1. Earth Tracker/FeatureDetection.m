%==========================================================================
% Niel Theron
% 16-07-2025
%==========================================================================
% The purpose of this function is to do feature detection
%==========================================================================
function [feature_pixel_locations, grayImage] = FeatureDetection(image,n_f,FD)

% Turn the image into gray scale
grayImage = rgb2gray(image);
%---

% Do feature detection
switch FD
    case 1
        points = detectSIFTFeatures(grayImage);
    case 2
        points = detectSURFFeatures(grayImage);
    case 3
        points = detectORBFeatures(grayImage);
    otherwise
        error('Invalid feature detection method specified.');
end
%---

% Get feature pixel locations
feature_pixels = points.selectStrongest(n_f);
feature_pixel_locations = feature_pixels.Location.';
%---

% Format output
if size(feature_pixel_locations,2) == n_f
    feature_pixel_locations = feature_pixels.Location.';
else
    feature_pixel_locations = zeros(2,n_f);
end
%---

end

