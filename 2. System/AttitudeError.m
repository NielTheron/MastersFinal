% 4 x N
q_EKF  = x_EKF(7:10, :);   % 4 x N
nSteps = size(q_EKF, 2);
q_true = x_true(7:10, 1:nSteps);
angle_deg = zeros(1, nSteps);

for k = 1:nSteps
    q_t = q_true(:, k)';   % 1x4 row
    q_e = q_EKF(:, k)';    % 1x4 row
    
    % Compute error quaternion: dq = q_true * conj(q_EKF)
    dq = quatmultiply(q_t, quatconj(q_e));
    dq = dq / norm(dq);  % normalize to avoid numerical errors
    
    % Full rotation angle (radians)
    angle_rad = 2 * acos(abs(dq(1)));  % scalar part of quaternion
    
    % Convert to degrees
    angle_deg(k) = rad2deg(angle_rad);
end

% Plot
figure;
yyaxis left
plot(time(1:nSteps), angle_deg, 'LineWidth', 1.2);
xlabel('Time Step');
ylabel('Angular Error (deg)');
title('Quaternion Directional Error Over Time');
grid on;

% Add right-side axis for arcseconds
yyaxis right
plot(time(1:nSteps), angle_deg * 3600, 'LineStyle', 'none'); % no extra line
ylabel('Angular Error (arcsec)');

% Error statistics
avg_error = mean(angle_deg);
max_error = max(angle_deg);
min_error = min(angle_deg);
std_error = std(angle_deg);

fprintf('Average Angular Error: %.6f deg (%.6f arcsec)\n', avg_error, avg_error*3600);
fprintf('Maximum Angular Error: %.6f deg (%.6f arcsec)\n', max_error, max_error*3600);
fprintf('Minimum Angular Error: %.6f deg (%.6f arcsec)\n', min_error, min_error*3600);
fprintf('Standard Deviation: %.6f deg (%.6f arcsec)\n', std_error, std_error*3600);
