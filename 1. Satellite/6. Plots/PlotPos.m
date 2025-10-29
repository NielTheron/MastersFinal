function PlotPos(xA,dt)
figure('Name','Real States')
n = (0:size(xA,2)-1)*dt;
tiledlayout(1,2)

nexttile
hold on
stairs(n,xA(1,:))
stairs(n,xA(2,:))
stairs(n,xA(3,:))
title('Real position')
xlabel('Time (s)')
ylabel("Position (ECI) (km)")
ylim([-8000 8000])
legend("rx","ry","rz")
grid on
hold off

nexttile
hold on
stairs(n,xA(4,:))
stairs(n,xA(5,:))
stairs(n,xA(6,:))
title('Real velocity')
xlabel('Time (s)')
ylabel("Velocity (ECI) (km/s)")
ylim([-8 8])
legend("vx","vy","vz")
grid on
hold off
end