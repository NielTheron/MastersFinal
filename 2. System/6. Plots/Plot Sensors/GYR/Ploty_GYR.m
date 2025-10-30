function Ploty_GYR(y_GYR,time)


figure('Name', 'Gyro Error', 'NumberTitle', 'off')

for i = 1:3
    subplot(3,1,i); % Stack the 3 plots vertically
    hold on

    % Logical mask to ignore samples where *all* GPS components are zero
    valid = any(y_GYR ~= 0, 1);  % True if any axis has a nonzero value

    % Filter data
    t_valid     = time(valid);
    y_valid     = y_GYR(i, valid);

    % Plot only valid data
    scatter(t_valid, y_valid, 'k', 'LineWidth', 1.2);

    xlabel('Time (s)');
    ylabel('Error');
    title(sprintf('Gyro Measurement (Axis %d)', i));
    legend('Error', 'Location', 'best');
    grid on;

    hold off
end

end
