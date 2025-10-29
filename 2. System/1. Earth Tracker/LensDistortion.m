%========================================================================
% Niel Theron
% 05-09-2025
%========================================================================
% Applies radial, tangential, and chromatic distortions to satellite images
%========================================================================
function distortedImage = LensDistortion(satelliteImage, ...
    radialDisPar, tangentialDisPar, chromaticDisPar, ...
    radial_enable, tangential_enable, chromatic_enable)

% Get image size
[height, width, channels] = size(satelliteImage);

% Generate normalized pixel coordinates (center at image center)
[X, Y] = meshgrid(1:width, 1:height);
x = (X - (width+1)/2) / (width/2);  % normalized [-1,1]
y = (Y - (height+1)/2) / (height/2);

% Initialize distorted coordinates
xDistorted = x;
yDistorted = y;

%% Radial distortion
if radial_enable
    k1 = radialDisPar(1); k2 = radialDisPar(2); k3 = radialDisPar(3);
    r2 = x.^2 + y.^2;
    radialFactor = 1 + k1*r2 + k2*r2.^2 + k3*r2.^3;
    xDistorted = xDistorted .* radialFactor;
    yDistorted = yDistorted .* radialFactor;
end

%% Tangential distortion
if tangential_enable
    p1 = tangentialDisPar(1); p2 = tangentialDisPar(2);
    r2 = x.^2 + y.^2;  % recompute if needed
    xTangential = 2*p1*x.*y + p2*(r2 + 2*x.^2);
    yTangential = p1*(r2 + 2*y.^2) + 2*p2*x.*y;
    xDistorted = xDistorted + xTangential;
    yDistorted = yDistorted + yTangential;
end

% Map normalized coordinates back to pixel coordinates
XDistorted = xDistorted * (width/2) + (width+1)/2;
YDistorted = yDistorted * (height/2) + (height+1)/2;

% Apply distortion to each channel
distortedImage = zeros(size(satelliteImage), 'like', satelliteImage);
for c = 1:channels
    distortedImage(:,:,c) = interp2(X, Y, double(satelliteImage(:,:,c)), ...
                                    XDistorted, YDistorted, 'linear', 0);
end

%% Chromatic aberration
if chromatic_enable
    distortedChromatic = distortedImage;

    scaleR = chromaticDisPar(1);  % shrink red
    scaleG = chromaticDisPar(2);  % keep green
    scaleB = chromaticDisPar(3);  % enlarge blue

    cx = (width+1)/2; cy = (height+1)/2;

    % Apply scaling for each channel
    XR = (X - cx) * scaleR + cx;
    YR = (Y - cy) * scaleR + cy;

    XG = (X - cx) * scaleG + cx;
    YG = (Y - cy) * scaleG + cy;

    XB = (X - cx) * scaleB + cx;
    YB = (Y - cy) * scaleB + cy;

    % Interpolate channels
    distortedChromatic(:,:,1) = interp2(X, Y, double(distortedChromatic(:,:,1)), XR, YR, 'linear', 0);
    distortedChromatic(:,:,2) = interp2(X, Y, double(distortedChromatic(:,:,2)), XG, YG, 'linear', 0);
    distortedChromatic(:,:,3) = interp2(X, Y, double(distortedChromatic(:,:,3)), XB, YB, 'linear', 0);

    distortedImage = cast(distortedChromatic, class(distortedImage));
end

end