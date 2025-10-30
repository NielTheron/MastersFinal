function Plotz_CSS(z_CSS, zhat_CSS, time)
% Plotz_GPS - Plots GPS measurements vs estimates, ignoring (0,0,0) entries.

figure('Name', 'CSS Measurements', 'NumberTitle', 'off')

for i = 1:3
    subplot(3,1,i); % Stack the 3 plots vertically
    hold on

    % Logical mask to ignore samples where *all* GPS components are zero
    valid = any(z_CSS ~= 0, 1);  % True if any axis has a nonzero value

    % Filter data
    t_valid     = time(valid);
    z_valid     = z_CSS(i, valid);
    zhat_valid  = zhat_CSS(i, valid);

    % Plot only valid data
    scatter(t_valid, z_valid, 'k', 'LineWidth', 1.2);
    scatter(t_valid, zhat_valid, 'r', 'LineWidth', 1.8);

    xlabel('Time (s)');
    ylabel('Magnitude');
    title(sprintf('CSS Measurement (Axis %d)', i));
    legend('True Value', 'Estimated Value', 'Location', 'best');
    grid on;

    hold off
end

end
