img = imread('10.png');

% Try mild tangential distortion
p1 = 0.00;   % horizontal skew
p2 = -0.05; % vertical skew

distorted = applyTangentialDistortion(img, p1, p2);

imshow(distorted);
title(sprintf('Tangential distortion p1=%.3f, p2=%.3f', p1, p2));



function distortedImg = applyTangentialDistortion(img, p1, p2)
    % APPLYTANGENTIALDISTORTION Applies tangential distortion to an image
    %   distortedImg = applyTangentialDistortion(img, p1, p2)
    %
    % Inputs:
    %   img - input image (RGB or grayscale)
    %   p1, p2 - tangential distortion coefficients
    %
    % Output:
    %   distortedImg - distorted image
    
    % Convert to double for safe interpolation
    img = im2double(img);
    [rows, cols, ch] = size(img);
    
    % Define image center
    cx = cols / 2;
    cy = rows / 2;
    [X, Y] = meshgrid(1:cols, 1:rows);
    
    % Normalized coordinates in [-1, 1]
    x_n = (X - cx) / cx;
    y_n = (Y - cy) / cy;
    
    % Radius squared
    r2 = x_n.^2 + y_n.^2;
    
    % Tangential distortion displacements
    x_t = p1 .* (r2 + 2*x_n.^2) + 2*p2 .* x_n .* y_n;
    y_t = 2*p1 .* x_n .* y_n + p2 .* (r2 + 2*y_n.^2);
    
    % Apply distortion
    x_distorted = x_n + x_t;
    y_distorted = y_n + y_t;
    
    % Back to pixel coordinates
    Xd = x_distorted * cx + cx;
    Yd = y_distorted * cy + cy;
    
    % Interpolate the distorted image
    distortedImg = zeros(size(img));
    for c = 1:ch
        distortedImg(:,:,c) = interp2(X, Y, img(:,:,c), Xd, Yd, 'linear', 0);
    end
end
