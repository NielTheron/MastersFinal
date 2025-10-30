%==========================================================================
% Niel Theron
% 03-10-2025
%==========================================================================
% The purpose of this function is to compare the z_ET measurement and the
% estimated zhat_ET measurement for multiple points
%==========================================================================

function SaveEstimatedImage(grayImage, z_ET, zhat_ET, Ix, Iy, ps, f, r)


%NOTE Might need to differentiate between ET and CAT focal length


% Input z_ET and zhat_ET are expected to be 3xN arrays

%=== T_B2C ================================================================
T_B2C = [-1   0   0   0;
          0  -1   0   0;
          0   0   1   0;
          0   0   0   1];

%=== T_M2P ================================================================
sx = 1/ps;
sy = 1/ps;
cx = Ix/2;
cy = Iy/2;
a = 0;

T_M2P = [sx     a       0       cx;
         0      -sy     0       cy;
         0      0       1       0;
         0      0       0       1];

% Convert grayscale back to RGB for annotation
rgbImage = repmat(grayImage, [1 1 3]);

%==========================================================================
% Loop through each feature
%==========================================================================

N = size(z_ET,2);  % number of points

for i = 1:N
    % Depths
    z  = z_ET(3,i);
    zh = zhat_ET(3,i);
    
    % Transform matrices (different per point because z differs)
    T_C2M_z  = [-f/z     0       0       0;
                 0      -f/z    0       0;
                 0       0     -f/z    0;
                 0       0      0       1];
    
    T_C2M_zh = [-f/zh    0       0       0;
                 0      -f/zh   0       0;
                 0       0     -f/zh   0;
                 0       0      0       1];
    
    % Transform measured point
    p_z = [z_ET(:,i);1];
    p_z = T_M2P * T_C2M_z * T_B2C * p_z;
    p_z = p_z(1:2);
    
    % Transform estimated point
    p_zh = [zhat_ET(:,i);1];
    p_zh = T_M2P * T_C2M_zh * T_B2C * p_zh;
    p_zh = p_zh(1:2);
    
    % Add measured z_ET (red circle)
    radii = 10;
    rgbImage = insertShape(rgbImage, 'Circle', [p_z(1), p_z(2), radii], ...
        'Color', 'red', 'LineWidth', 3);
    
    % Add estimated zhat_ET (blue cross)
    crossSize = 15;   % half-length of arms
    lineWidth = 5;    % thickness

    % Draw lines for the cross
    x = p_zh(1); y = p_zh(2);

    rgbImage = insertShape(rgbImage, 'Line', ...
        [x-crossSize, y-crossSize, x+crossSize, y+crossSize], ...
        'Color', 'blue', 'LineWidth', lineWidth);

    rgbImage = insertShape(rgbImage, 'Line', ...
        [x-crossSize, y+crossSize, x+crossSize, y-crossSize], ...
        'Color', 'blue', 'LineWidth', lineWidth);
end

%==========================================================================
% Save image
%==========================================================================
filename = sprintf('estimated_image_%03d.png', r);  
outputFolder = 'C:\Users\Niel\Desktop\Masters\2. System\1. Earth Tracker\EstimatedImages';
fullpath = fullfile(outputFolder, filename);
imwrite(rgbImage, fullpath);

end
