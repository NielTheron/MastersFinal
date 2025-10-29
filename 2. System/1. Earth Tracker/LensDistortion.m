function distortedImage = LensDistortion(satelliteImage, ...
    radialDisPar, tangentialDisPar, chromaticDisPar, ...
    radial_enable, tangential_enable, chromatic_enable)
%========================================================================
% Applies radial, tangential, and chromatic distortions to satellite images
% Order: Chromatic → Geometric (physically more accurate)
%========================================================================

% Get image size
[height, width, channels] = size(satelliteImage);

% Generate normalized pixel coordinates (center at image center)
[X, Y] = meshgrid(1:width, 1:height);
cx = (width+1)/2;
cy = (height+1)/2;
x = (X - cx) / (width/2);  % normalized [-1,1]
y = (Y - cy) / (height/2);

%% Apply Chromatic Aberration FIRST (if enabled)
if chromatic_enable && channels == 3
    chromaticImage = zeros(size(satelliteImage), 'like', satelliteImage);
    
    scaleR = chromaticDisPar(1);  % shrink red
    scaleG = chromaticDisPar(2);  % keep green
    scaleB = chromaticDisPar(3);  % enlarge blue
    
    % Apply scaling for each channel
    XR = (X - cx) * scaleR + cx;
    YR = (Y - cy) * scaleR + cy;
    chromaticImage(:,:,1) = interp2(X, Y, double(satelliteImage(:,:,1)), XR, YR, 'linear', 0);
    
    XG = (X - cx) * scaleG + cx;
    YG = (Y - cy) * scaleG + cy;
    chromaticImage(:,:,2) = interp2(X, Y, double(satelliteImage(:,:,2)), XG, YG, 'linear', 0);
    
    XB = (X - cx) * scaleB + cx;
    YB = (Y - cy) * scaleB + cy;
    chromaticImage(:,:,3) = interp2(X, Y, double(satelliteImage(:,:,3)), XB, YB, 'linear', 0);
    
    satelliteImage = cast(chromaticImage, class(satelliteImage));
end

%% Apply Geometric Distortions
% Initialize distorted coordinates
xDistorted = x;
yDistorted = y;

% Compute r² once (used by both distortions)
r2 = x.^2 + y.^2;

% Radial distortion
if radial_enable
    k1 = radialDisPar(1); 
    k2 = radialDisPar(2); 
    k3 = radialDisPar(3);
    
    radialFactor = 1 + k1*r2 + k2*r2.^2 + k3*r2.^3;
    xDistorted = xDistorted .* radialFactor;
    yDistorted = yDistorted .* radialFactor;
end

% Tangential distortion
if tangential_enable
    p1 = tangentialDisPar(1); 
    p2 = tangentialDisPar(2);
    
    xTangential = 2*p1*x.*y + p2*(r2 + 2*x.^2);
    yTangential = p1*(r2 + 2*y.^2) + 2*p2*x.*y;
    xDistorted = xDistorted + xTangential;
    yDistorted = yDistorted + yTangential;
end

% Map normalized coordinates back to pixel coordinates
XDistorted = xDistorted * (width/2) + cx;
YDistorted = yDistorted * (height/2) + cy;

% Apply geometric distortion to each channel
distortedImage = zeros(size(satelliteImage), 'like', satelliteImage);
for c = 1:channels
    distortedImage(:,:,c) = interp2(X, Y, double(satelliteImage(:,:,c)), ...
                                    XDistorted, YDistorted, 'linear', 0);
end

end