function PlotStateError(xA,x_est,dt)
figure('Name','Position Error')
n = (0:size(x_est,2)-1)*dt;

tiledlayout(2,2)
nexttile
hold on
stairs(n,(xA(1,1:length(n))-x_est(1,1:length(n)))*1000)
stairs(n,(xA(2,1:length(n))-x_est(2,1:length(n)))*1000)
stairs(n,(xA(3,1:length(n))-x_est(3,1:length(n)))*1000)
title('Position Error')
xlabel('Time (s)')
ylabel("Position (ECI) (m)")
grid on
hold off

nexttile
hold on
stairs(n,(xA(4,1:length(n))-x_est(4,1:length(n)))*1000)
stairs(n,(xA(5,1:length(n))-x_est(5,1:length(n)))*1000)
stairs(n,(xA(6,1:length(n))-x_est(6,1:length(n)))*1000)
title('Velocity Error')
xlabel('Time (s)')
ylabel("Velocity (ECI) (m/s)")
grid on
hold off

nexttile
hold on
stairs(n,(xA(7,1:length(n))-x_est(7,1:length(n))))
stairs(n,(xA(8,1:length(n))-x_est(8,1:length(n))))
stairs(n,(xA(9,1:length(n))-x_est(9,1:length(n))))
stairs(n,(xA(10,1:length(n))-x_est(10,1:length(n))))
title('Attitude Error')
xlabel('Time (s)')
ylabel("Attitude")
grid on
hold off

% nexttile
% hold on
% % Vectorized version
% dot_products = sum(xA(7:10,:) .* x_est(7:10,:), 1); % Dot product along rows
% dot_products = max(-1, min(1, abs(dot_products)));   % Clamp
% angular_error = 2 * acos(dot_products);
% stairs(n, rad2deg(angular_error));
% title('Attitude Error (deg)')
% xlabel('Time (s)')
% ylabel("Attitude (BOD2ECI)")
% grid on
% hold off

nexttile
hold on
stairs(n,rad2deg(xA(11,1:length(n))-x_est(11,1:length(n))))
stairs(n,rad2deg(xA(12,1:length(n))-x_est(12,1:length(n))))
stairs(n,rad2deg(xA(13,1:length(n))-x_est(13,1:length(n))))
title('Angular Velocity Error')
xlabel('Time (s)')
ylabel("Angular Velocity (BOD) (deg/s)")
grid on
hold off
end