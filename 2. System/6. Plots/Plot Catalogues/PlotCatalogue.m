function PlotCatalogue(satelliteImage, featureLocations, catalogueLocations, frameIdx)
% PlotCatalogue - Display two images:
%   1. Feature pixel coordinates (x, y)
%   2. Catalogue geolocations (lat°, lon°, alt km) at the same pixel positions
%
% Inputs:
%   satelliteImage      - [720 x 720 x 3 x N] uint8 satellite images
%   featureLocations    - [2 x M x N] feature pixel locations (x, y)
%   catalogueLocations  - [3 x M x N] catalogue geolocations (lat, lon, alt)
%   frameIdx            - (optional) frame index to display, default = 1

    if nargin < 4
        frameIdx = 1;
    end

    % Extract frame data
    img = satelliteImage(:,:,:,frameIdx);
    feat = featureLocations(:,:,frameIdx);
    cat  = catalogueLocations(:,:,frameIdx);

    %% ---- First figure: Feature pixel coordinates ----
    figure('Name', sprintf('Frame %d - Feature Pixel Coordinates', frameIdx));
    imshow(img);
    hold on;

    plot(feat(1,:), feat(2,:), 'ro', 'MarkerSize', 6, 'LineWidth', 1.5);
    for i = 1:size(feat,2)
        x = feat(1,i);
        y = feat(2,i);
        text(x+4, y, sprintf('(%.0f, %.0f)', x, y), ...
            'Color', 'r', 'FontSize', 16, 'FontWeight', 'bold', ...
            'BackgroundColor', 'k', 'Margin', 1);
    end

    legend('Feature Pixels (x,y)');
    %title(sprintf('Frame %d - Feature Pixel Coordinates (x, y)', frameIdx));
    hold off;

    %% ---- Second figure: Catalogue geolocations ----
    figure('Name', sprintf('Frame %d - Catalogue Geolocations', frameIdx));
    imshow(img);
    hold on;

    plot(feat(1,:), feat(2,:), 'bs', 'MarkerSize', 6, 'LineWidth', 1.5);
    for i = 1:size(feat,2)
        lat = cat(1,i);
        lon = cat(2,i);
        text(feat(1,i)+4, feat(2,i), sprintf('(%.2f°, %.2f°)', lat, lon), ...
            'Color', 'y', 'FontSize', 16, 'FontWeight', 'bold', ...
            'BackgroundColor', 'k', 'Margin', 1);
    end

    legend('Catalogue (Lat, Lon)');
    %title(sprintf('Frame %d - Catalogue Geolocations (Lat°, Lon°)', frameIdx));
    hold off;

end
