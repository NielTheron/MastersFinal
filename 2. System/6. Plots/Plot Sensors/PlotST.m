function PlotST(z_ST_all, dt)
%==========================================================================
% Plot Star Tracker Measurements for the first 4 stars
% Each star is plotted in a separate subplot as scatter plot
% Ignores zero measurements
%==========================================================================
% z_ST_all : 3 x N       or 3 x N x M array
% dt       : time step (s)
%==========================================================================

dims = size(z_ST_all);
nStars = dims(2);
nStarsToPlot = min(4, nStars); % only plot up to 4 stars

if numel(dims) < 3
    nTimes = 1;  % single timestep
    z_ST_all = reshape(z_ST_all, [3, nStars, 1]);
else
    nTimes = dims(3);
end

% Time vector
t = (0:nTimes-1) * dt;

% Colors and component labels
colors = {'b','g','r'};
compLabels = {'x','y','z'};

figure('Name', 'Star Tracker Measurements - First 4 Stars');

for s = 1:nStarsToPlot
    subplot(nStarsToPlot,1,s);
    hold on;
    
    % Extract measurements for this star
    star_vecs = squeeze(z_ST_all(:,s,:)); % 3 x nTimes
    
    for c = 1:3
        % Find non-zero measurements
        valid_idx = star_vecs(c,:) ~= 0;
        scatter(t(valid_idx), star_vecs(c,valid_idx), 20, colors{c}, 'filled', ...
            'DisplayName', sprintf('%s component', compLabels{c}));
    end
    
    title(sprintf('Star %d Components', s));
    xlabel('Time (s)');
    ylabel('Unit Vector Component');
    legend('show');
    grid on;
    hold off;
end

end
