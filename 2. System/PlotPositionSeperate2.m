function PlotPositionSeperate2(x_true, x_est, n_s, dt)
figure('Name', 'Position Plots (Stacked)');
t = (0:n_s-1) * dt;

for i = 1:3
    subplot(3,1,i);
    
    % Main plot
    plot(t, x_true(i,:), 'k', 'LineWidth', 1.2);
    hold on
    plot(t, x_est(i,:), 'r--', 'LineWidth', 1.2);
    xlabel('Time (s)');
    ylabel('Position (km)');
    title(sprintf('Position (Axis %d)', i));
    legend('True', 'Estimated', 'Location', 'best');
    grid on;
    
    % Force y-axis limits to show variation
    y_mean = mean([x_true(i,:), x_est(i,:)]);
    y_range = max(abs([x_true(i,:) - y_mean, x_est(i,:) - y_mean]));
    ylim([y_mean - 2*y_range, y_mean + 2*y_range]);
end
end