%=======================================================================
% Niel Theorn
% 09-05-2025
%=======================================================================

function z_ET = EarthTracker(features,Ix,Iy,focalLength,pixelSize,alpha,noise_ET)


%=== T_P2M =============================================================
cx = Ix/2;
cy = Iy/2;

T_P2M = [pixelSize      alpha           0              -cx*pixelSize;
         0             -pixelSize       0               cy*pixelSize;
         0              0              -focalLength     0;
         0              0               0               1];

%f_M = T_P2M * [features; 1; 1]
%---

%=== T_M2C =============================================================

s = 500;
T_M2C = [-s/focalLength   0               0               0;
          0              -s/focalLength   0               0;
          0               0              -s/focalLength   0;
          0               0               0               1];

%f_C = T_M2C * f_M
%----

%=== T_C2B =============================================================

T_C2B = [-1   0   0   0;
          0  -1   0   0;
          0   0   1   0;
          0   0   0   1];

% f_B = T_C2B * f_C
%----

% Create Measurment for all features -----------------------------------
numFeatures = size(features, 2);
z_ET = zeros(3, numFeatures);

for i = 1:numFeatures
    f_P = [features(:,i); 1; 1]; % Homogeneous pixel vector;
    f_I = T_C2B * T_M2C * T_P2M * f_P;
    % noise_ET = noise_ET.*randn(3,1);
    z_ET(:,i) = f_I(1:3);% + noise_ET/1000;
end
%---

end