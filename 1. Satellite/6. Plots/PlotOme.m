function PlotOme(xA,dt)
figure('Name','Real States')

n = (0:size(xA,2)-1)*dt;
tiledlayout(1,1)

nexttile
hold on
stairs(n,xA(11,:))
stairs(n,xA(12,:))
stairs(n,xA(13,:))
title('Real angular velocity')
xlabel('Time (s)')
ylabel("Angular velocity (BOD) (rad/s)")
ylim([-1 1])
legend("wx","wy","wz")
grid on
hold off
end