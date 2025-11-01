function PlotH_GPS(H)
% Assuming H is a 3x13xT array
T = size(H, 3);

% State labels
state_labels = {'rx', 'ry', 'rz', 'vx', 'vy', 'vz', 'qs', 'qx', 'qy', 'qz', 'wx', 'wy', 'wz'};

% Measurement labels (assuming 3 measurements)
meas_labels = {'Lat', 'Lon', 'Alt'};

% Setup video writer
v = VideoWriter('H_GPS_matrix_animation.mp4', 'MPEG-4');
v.FrameRate = 10;
v.Quality = 95;
open(v);

% Create figure
fig = figure('Color', 'w', 'Position', [100, 100, 1200, 400]);

for t = 1:T
    % Extract H matrix at current time
    H_t = abs(H(:, :, t));  % 3x13
    
    % Skip this frame if H is all zeros (no measurement)
    if all(H_t(:) == 0)
        continue;
    end
    
    % Calculate dynamic color limits for current time step
    min_val = min(H_t(:));
    max_val = max(H_t(:));
    
    % Handle case where all values are identical (but not zero)
    if min_val == max_val
        clim_H = [min_val - 1e-10, max_val + 1e-10];
    else
        clim_H = [min_val, max_val];
    end
    
    % Plot H matrix
    imagesc(H_t);
    colorbar;
    title(sprintf('Measurement Jacobian (H) - t=%d/%d', t, T));
    ylabel('Measurements');
    xlabel('States');
    set(gca, 'XTick', 1:13, 'XTickLabel', state_labels, 'XTickLabelRotation', 45);
    set(gca, 'YTick', 1:3, 'YTickLabel', meas_labels);
    clim(clim_H);
    
    % Force drawing to complete
    drawnow;
    
    % Capture frame and write to video
    frame = getframe(fig);
    writeVideo(v, frame);
end

% Close video file
close(v);
close(fig);

fprintf('Video saved as H_GPS_matrix_animation.mp4\n');
end