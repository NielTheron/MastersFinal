function PlotState(xA,x_est,dt)
figure('Name','Real States')

n = (0:size(x_est,2)-1)*dt;
tiledlayout(2,2)

nexttile
hold on
stairs(n,xA(1,1:length(n)))
stairs(n,xA(2,1:length(n)))
stairs(n,xA(3,1:length(n)))
title('Real position')
xlabel('Time (s)')
ylabel("Position (ECI) (km)")
ylim([-8000 8000])
grid on
hold off

nexttile
hold on
stairs(n,xA(4,1:length(n)))
stairs(n,xA(5,1:length(n)))
stairs(n,xA(6,1:length(n)))
title('Real velocity')
xlabel('Time (s)')
ylabel("Velocity (ECI) (km/s)")
ylim([-8 8])
grid on
hold off

nexttile
hold on
stairs(n,xA(7,1:length(n)))
stairs(n,xA(8,1:length(n)))
stairs(n,xA(9,1:length(n)))
stairs(n,xA(10,1:length(n)))
title('Real attitude')
xlabel('Time (s)')
ylabel("Attitude (BOD2ECI)")
ylim([-2 2])
grid on
hold off

nexttile
hold on
stairs(n,rad2deg(xA(11,1:length(n))))
stairs(n,rad2deg(xA(12,1:length(n))))
stairs(n,rad2deg(xA(13,1:length(n))))
title('Real angular velocity')
xlabel('Time (s)')
ylabel("Angular velocity (BOD) (deg/s)")
% ylim([-1 1])
grid on
hold off
end