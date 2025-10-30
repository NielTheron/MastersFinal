function Plotz_ET(z_ET, n_f, dt)
    figure('Name', 'Real Measurement')
    hold on
    n = (0:size(z_ET,3)-1) * dt;

    % Axis colors and labels
    colors = {'b', 'g', 'r'};
    labels = {'x', 'y', 'z'};

    for i = 1:n_f
        for k = 1:3  % x, y, z components
            y_vals = squeeze(z_ET(k, i, :))';  % Get 1×time vector
            valid_idx = y_vals ~= 0;
            n_valid = n(valid_idx);
            y_valid = y_vals(valid_idx);

            % Scatter plot for non-zero measurements
            scatter(n_valid, y_valid, 20, colors{k}, 'filled', ...
                'DisplayName', sprintf('Feature %d %s', i, labels{k}));
        end
    end

    title("Real Measurements")
    xlabel("Time (s)")
    ylabel("Distance (km) in Body Frame")
    % legend('show')
    grid on
    hold off
end