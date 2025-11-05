function PlotStateError(x_true, x_est, dt)
figure('Name','State Error')
n = (0:size(x_est,2)-1) * dt;

tiledlayout(2,2)

%% ------------------------------------------------------------------------
% POSITION ERROR
nexttile
hold on
stairs(n,(x_true(1,:) - x_est(1,:)) * 1000)
stairs(n,(x_true(2,:) - x_est(2,:)) * 1000)
stairs(n,(x_true(3,:) - x_est(3,:)) * 1000)
title('Position Error')
xlabel('Time (s)')
ylabel('Position (ECI) (m)')
grid on
hold off

%% ------------------------------------------------------------------------
% VELOCITY ERROR
nexttile
hold on
stairs(n,(x_true(4,:) - x_est(4,:)) * 1000)
stairs(n,(x_true(5,:) - x_est(5,:)) * 1000)
stairs(n,(x_true(6,:) - x_est(6,:)) * 1000)
title('Velocity Error')
xlabel('Time (s)')
ylabel('Velocity (ECI) (m/s)')
grid on
hold off

%% ------------------------------------------------------------------------
% ATTITUDE ERROR (Quaternion Angle Difference)
nexttile
hold on
angular_error = zeros(1, length(n));

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

stairs(n, angular_error, 'LineWidth', 1.2)
title('Attitude Error (deg)')
xlabel('Time (s)')
ylabel('Rotation Angle Difference (°)')
grid on
hold off

%% ------------------------------------------------------------------------
% ANGULAR VELOCITY ERROR
nexttile
hold on
stairs(n, rad2deg(x_true(11,:) - x_est(11,:)))
stairs(n, rad2deg(x_true(12,:) - x_est(12,:)))
stairs(n, rad2deg(x_true(13,:) - x_est(13,:)))
title('Angular Velocity Error')
xlabel('Time (s)')
ylabel('Angular Velocity (deg/s)')
grid on
hold off
end
