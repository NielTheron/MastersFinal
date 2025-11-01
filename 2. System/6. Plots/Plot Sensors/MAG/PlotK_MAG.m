function PlotK_MAG(K)
% Assuming K is a 13x3xT array
T = size(K, 3);

% State labels
state_labels = {'rx', 'ry', 'rz', 'vx', 'vy', 'vz', 'qs', 'qx', 'qy', 'qz', 'wx', 'wy', 'wz'};

% Measurement labels (assuming 3 measurements)
meas_labels = {'bx', 'by', 'bz'};

% Setup video writer
v = VideoWriter('K_MAG_matrix_animation.mp4', 'MPEG-4');
v.FrameRate = 10;
v.Quality = 95;
open(v);

% Create figure - taller for vertical orientation
fig = figure('Color', 'w', 'Position', [100, 100, 600, 1000]);

for t = 1:T
    % Extract K matrix at current time
    K_t = abs(K(:, :, t));  % 13x3
    
    % Skip this frame if K is all zeros (no measurement update)
    if all(K_t(:) == 0)
        continue;
    end
    
    % Calculate dynamic color limits for current time step
    min_val = min(K_t(:));
    max_val = max(K_t(:));
    
    % Handle case where all values are identical (but not zero)
    if min_val == max_val
        clim_K = [min_val - 1e-10, max_val + 1e-10];
    else
        clim_K = [min_val, max_val];
    end
    
    % Plot K matrix
    imagesc(K_t);
    colorbar;
    title(sprintf('Kalman Gain (K) - t=%d/%d', t, T));
    ylabel('States');
    xlabel('Measurements');
    set(gca, 'XTick', 1:3, 'XTickLabel', meas_labels);
    set(gca, 'YTick', 1:13, 'YTickLabel', state_labels);
    clim(clim_K);
    
    % Force drawing to complete
    drawnow;
    
    % Capture frame and write to video
    frame = getframe(fig);
    writeVideo(v, frame);
end

% Close video file
close(v);
close(fig);

fprintf('Video saved as K_MAG_matrix_animation.mp4\n');
end