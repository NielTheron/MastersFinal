function PlotP(P)
% Assuming P is a 13x13xT array
T = size(P, 3);

% Define state indices for rows (your state groups)
idx_state1 = 1:3;      % Position (rx, ry, rz)
idx_state2 = 4:6;      % Velocity (vx, vy, vz)
idx_state3 = 7:10;     % Quaternion (qs, qx, qy, qz)
idx_state4 = 11:13;    % Angular velocity (wx, wy, wz)

% All columns (all 13 states)
idx_all = 1:13;

% State labels
state_labels = {'rx', 'ry', 'rz', 'vx', 'vy', 'vz', 'qs', 'qx', 'qy', 'qz', 'wx', 'wy', 'wz'};

% Setup video writer
v = VideoWriter('P_matrix_animation.mp4', 'MPEG-4');
v.FrameRate = 10;
v.Quality = 95;
open(v);

% Create figure - taller for vertical stacking
fig = figure('Color', 'w', 'Position', [100, 100, 1200, 1200]);

for t = 1:T
    % Extract blocks (each row group vs all columns)
    P1 = abs(P(idx_state1, idx_all, t));  % 3x13
    P2 = abs(P(idx_state2, idx_all, t));  % 3x13
    P3 = abs(P(idx_state3, idx_all, t));  % 4x13
    P4 = abs(P(idx_state4, idx_all, t));  % 3x13
    
    % Calculate dynamic color limits for current time step
    clim1 = [min(P1(:)), max(P1(:))];
    clim2 = [min(P2(:)), max(P2(:))];
    clim3 = [min(P3(:)), max(P3(:))];
    clim4 = [min(P4(:)), max(P4(:))];
    
    % Subplot 1 (top) - Position
    subplot(4, 1, 1);
    imagesc(P1);
    colorbar;
    title(sprintf('Position Covariance - t=%d/%d', t, T));
    ylabel('Position');
    xlabel('All States');
    set(gca, 'XTick', 1:13, 'XTickLabel', state_labels, 'XTickLabelRotation', 45);
    set(gca, 'YTick', 1:3, 'YTickLabel', state_labels(idx_state1));
    clim(clim1);
    
    % Subplot 2 - Velocity
    subplot(4, 1, 2);
    imagesc(P2);
    colorbar;
    title('Velocity Covariance');
    ylabel('Velocity');
    xlabel('All States');
    set(gca, 'XTick', 1:13, 'XTickLabel', state_labels, 'XTickLabelRotation', 45);
    set(gca, 'YTick', 1:3, 'YTickLabel', state_labels(idx_state2));
    clim(clim2);
    
    % Subplot 3 - Quaternion
    subplot(4, 1, 3);
    imagesc(P3);
    colorbar;
    title('Quaternion Covariance');
    ylabel('Quaternion');
    xlabel('All States');
    set(gca, 'XTick', 1:13, 'XTickLabel', state_labels, 'XTickLabelRotation', 45);
    set(gca, 'YTick', 1:4, 'YTickLabel', state_labels(idx_state3));
    clim(clim3);
    
    % Subplot 4 (bottom) - Angular Velocity
    subplot(4, 1, 4);
    imagesc(P4);
    colorbar;
    title('Angular Velocity Covariance');
    ylabel('Angular Vel');
    xlabel('All States');
    set(gca, 'XTick', 1:13, 'XTickLabel', state_labels, 'XTickLabelRotation', 45);
    set(gca, 'YTick', 1:3, 'YTickLabel', state_labels(idx_state4));
    clim(clim4);
    
    % Force drawing to complete
    drawnow;
    
    % Capture frame and write to video
    frame = getframe(fig);
    writeVideo(v, frame);
end

% Close video file
close(v);
close(fig);

fprintf('Video saved as P_matrix_animation.mp4\n');
end