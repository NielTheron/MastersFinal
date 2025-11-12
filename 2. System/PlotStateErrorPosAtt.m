function PlotStateErrorPosAtt(x_true, x_est, dt)
% Create figure with specific size
fig = figure('Name','State Error', 'NumberTitle','off');
fig.Position = [100, 100, 1200, 500];  % [left, bottom, width, height] in pixels

n = (0:size(x_est,2)-1) * dt;
d = size(x_est,2);

tiledlayout(1,2)

%% ------------------------------------------------------------------------
% POSITION ERROR
nexttile
hold on
stairs(n,(x_true(1,1:d) - x_est(1,:)) * 1000)
stairs(n,(x_true(2,1:d) - x_est(2,:)) * 1000)
stairs(n,(x_true(3,1:d) - x_est(3,:)) * 1000)
title('Position Error')
xlabel('Time (s)')
ylabel('Position (m)')
legend('x-Along Track','y-Cross Track','z-Nadir','Location','best')
grid on
hold off

%% ------------------------------------------------------------------------
% ATTITUDE ERROR (Quaternion Angle Difference)
nexttile
hold on
angular_error = zeros(1, d);

for k = 1:length(n)
    q_true = x_true(7:10, k);
    q_est  = x_est(7:10, k);

    % Normalize to avoid drift
    q_true = q_true / norm(q_true);
    q_est  = q_est / norm(q_est);

    % Quaternion conjugate of estimate
    q_est_conj = [q_est(1); -q_est(2:4)];

    % Quaternion error: q_err = q_true * conj(q_est)
    q_err = quatmultiply(q_true', q_est_conj');

    % Angle error in radians
    theta = 2 * acos(min(1, abs(q_err(1))));
    angular_error(k) = rad2deg(theta);
end

yyaxis left
stairs(n, angular_error, 'LineWidth', 1.2)
ylabel('Rotation Angle Difference (°)')

yyaxis right
% Make right axis invisible, but keep labels
stairs(n, angular_error, 'LineStyle', 'none') 
ax = gca;

% Force right axis limits to match left
ax.YLim = ax.YAxis(1).Limits * 3600;

% Set right axis ticks based on left axis ticks
ax.YAxis(2).TickValues = ax.YAxis(1).TickValues * 3600;
ax.YAxis(2).Label.String = 'Rotation Angle Difference (arcsec)';

title('Attitude Error')
xlabel('Time (s)')
grid on
hold off
end
