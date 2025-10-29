function PlotAttitudeSeperate(x_true, x_est, P, n_s, dt)

figure('Name', 'Attitude Plots (Stacked)');

% Create time vector in seconds
t = (0:n_s-1) * dt;

for i = 1:4
    subplot(4,1,i); % Stack the 3 plots vertically

    err = squeeze(sqrt(P(i+6,i+6,:)))';
    x_patch = [t, fliplr(t)];
    y_patch = [x_est(i+6,:) - err, fliplr(x_est(i+6,:) + err)];

    hold on
    patch(x_patch, y_patch, 'b', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    plot(t, x_true(i+6,:), 'k', 'LineWidth', 1.2);
    plot(t, x_est(i+6,:), 'r', 'LineWidth', 1.8);
    hold off

    xlabel('Time (s)');
    ylabel('Magnitude');
    title(sprintf('Attitude Estimate with Variance (Axis %d)', i));
    legend('±1σ Bound', 'True Value', 'Estimated Value', 'Location', 'best');
    grid on;
end

end