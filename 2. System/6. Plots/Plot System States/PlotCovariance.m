function PlotCovariance(xA,x_est,Pp,n_s)

figure(Name="Position plot")
for i = 1:3

    err = squeeze(sqrt(Pp(i,i,:)))';

    hold on
    plot(xA(i,:))
    plot(x_est(i,:),LineWidth=2)
    t = linspace(1,n_s,n_s);
    x_patch = [t, fliplr(t)];
    y_patch = [x_est(i,:) - err, fliplr(x_est(i,:) + err)];
    patch(x_patch, y_patch, 'b', 'FaceAlpha',0.5, 'EdgeColor','none')
    xlabel('Time');
    ylabel('Estimated Value');
    title('Estimate with Variance');
    grid on
    hold off
end

figure(Name="Velocity plot")
for i = 4:6

    err = squeeze(sqrt(Pp(i,i,:)))';

    hold on
    plot(xA(i,:))
    plot(x_est(i,:),LineWidth=2)
    t = linspace(1,n_s,n_s);
    x_patch = [t, fliplr(t)];
    y_patch = [x_est(i,:) - err, fliplr(x_est(i,:) + err)];
    patch(x_patch, y_patch, 'b', 'FaceAlpha',0.5, 'EdgeColor','none')
    xlabel('Time');
    ylabel('Estimated Value');
    title('Estimate with Variance');
    grid on
    hold off
end

figure(Name="Attitude plot")
for i = 7:10

    err = squeeze(sqrt(Pp(i,i,:)))';

    hold on
    plot(xA(i,:))
    plot(x_est(i,:),LineWidth=2)
    t = linspace(1,n_s,n_s);
    x_patch = [t, fliplr(t)];
    y_patch = [x_est(i,:) - err, fliplr(x_est(i,:) + err)];
    % patch(x_patch, y_patch, 'b', 'FaceAlpha',0.5, 'EdgeColor','none')
    xlabel('Time');
    ylabel('Estimated Value');
    title('Estimate with Variance');
    grid on
    hold off
end

figure(Name="Angular Velocity plot")
for i = 11:13

    err = squeeze(sqrt(Pp(i,i,:)))';

    hold on
    plot(xA(i,:))
    plot(x_est(i,:),LineWidth=2)
    t = linspace(1,n_s,n_s);
    x_patch = [t, fliplr(t)];
    y_patch = [x_est(i,:) - err, fliplr(x_est(i,:) + err)];
    patch(x_patch, y_patch, 'b', 'FaceAlpha',0.5, 'EdgeColor','none')
    xlabel('Time');
    ylabel('Estimated Value');
    title('Estimate with Variance');
    grid on
    hold off
end


end