function PlotAngularSeperate(x_true, x_est, P, n_s, dt)

figure('Name', 'Angular Velocity Plots (Stacked)');

% Create time vector in seconds
t = (0:n_s-1) * dt;

for i = 1:3
    subplot(3,1,i); % Stack the 3 plots vertically

    err = squeeze(rad2deg(sqrt(P(i+10,i+10,:))))';
    x_patch = [t, fliplr(t)];
    y_patch = [rad2deg(x_est(i+10,:)) - err, fliplr(rad2deg(x_est(i+10,:)) + err)];

    hold on
    patch(x_patch, y_patch, 'b', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    plot(t, rad2deg(x_true(i+10,:)), 'k', 'LineWidth', 1.2);
    plot(t, rad2deg(x_est(i+10,:)), 'r', 'LineWidth', 1.8);
    hold off

    xlabel('Time (s)');
    ylabel('Angular Velocity (deg/s)');
    title(sprintf('Angular Velocity Estimate with Variance (Axis %d)', i));
    legend('±1σ Bound', 'True Value', 'Estimated Value', 'Location', 'best');
    grid on;
end

end