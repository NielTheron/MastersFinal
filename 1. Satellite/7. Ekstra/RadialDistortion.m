
img = imread('10.png');
distorted = applyRadialDistortion(img, 0.5, -0.0, 0.0);
imshow(distorted);

function distortedImg = applyRadialDistortion(img, k1, k2, k3)
    % APPLYRADIALDISTORTION Applies radial distortion to an image
    %   distortedImg = applyRadialDistortion(img, k1, k2, k3)
    %
    % Inputs:
    %   img - input image (RGB or grayscale)
    %   k1, k2, k3 - radial distortion coefficients
    %
    % Output:
    %   distortedImg - distorted image

    % Convert to double precision for interpolation
    img = im2double(img);
    [rows, cols, ch] = size(img);

    % Define normalized coordinates in range [-1, 1]
    cx = cols / 2;
    cy = rows / 2;
    [X, Y] = meshgrid(1:cols, 1:rows);

    % Normalize coordinates
    x_n = (X - cx) / cx;  
    y_n = (Y - cy) / cy;

    % Compute radius squared
    r2 = x_n.^2 + y_n.^2;

    % Compute radial distortion term
    radial = k1 * r2 + k2 * r2.^2 + k3 * r2.^3;

    % Distortion displacement (your equation)
    x_r = x_n .* radial;
    y_r = y_n .* radial;

    % Apply distortion: new coords = original + displacement
    x_distorted = x_n + x_r;
    y_distorted = y_n + y_r;

    % Convert back to pixel coordinates
    Xd = x_distorted * cx + cx;
    Yd = y_distorted * cy + cy;

    % Interpolate
    distortedImg = zeros(size(img));
    for c = 1:ch
        distortedImg(:,:,c) = interp2(X, Y, img(:,:,c), Xd, Yd, 'linear', 0);
    end
end
