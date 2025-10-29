function PlotJ2(xA,xO, dt)
figure('Name','Real States')
n = (0:size(xA,2)-1)*dt;
tiledlayout(1,2)

nexttile
hold on
stairs(n,xO(1,:) - xA(1,:))
stairs(n,xO(2,:) - xA(2,:))
stairs(n,xO(3,:) - xA(3,:))
title('Real position')
xlabel('Time (s)')
ylabel("Position Error (ECI) (km)")
ylim([-60 60])
legend("rx","ry","rz")
grid on
hold off

nexttile
hold on
stairs(n,xO(4,:) - xA(4,:))
stairs(n,xO(5,:) - xA(5,:))
stairs(n,xO(6,:) - xA(6,:))
title('Real velocity')
xlabel('Time (s)')
ylabel("Velocity Error (ECI) (km/s)")
ylim([-0.06 0.06])
legend("vx","vy","vz")
grid on
hold off
end