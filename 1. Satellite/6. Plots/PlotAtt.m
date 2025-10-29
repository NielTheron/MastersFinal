function PlotAtt(xA,xO,dt)
figure('Name','Real States')

n = (0:size(xA,2)-1)*dt;
tiledlayout(1,3)

nexttile
hold on
stairs(n,xA(7,:))
stairs(n,xA(8,:))
stairs(n,xA(9,:))
stairs(n,xA(10,:))
title('Real attitude')
xlabel('Time (s)')
ylabel("Attitude (BOD)")
ylim([-2 2])
legend("qs","qx","qy","qz")
grid on
hold off

nexttile
hold on
stairs(n,xO(7,:))
stairs(n,xO(8,:))
stairs(n,xO(9,:))
stairs(n,xO(10,:))
title('Attitude with Coupling')
xlabel('Time (s)')
ylabel("Attitude (BOD)")
ylim([-2 2])
legend("qs","qx","qy","qz")
grid on
hold off

nexttile
hold on
stairs(n,xO(7,:) - xA(7,:))
stairs(n,xO(8,:) - xA(8,:))
stairs(n,xO(9,:) - xA(9,:))
stairs(n,xO(10,:) - xA(10,:))
title('Attitude Error')
xlabel('Time (s)')
ylabel("Attitude (BOD)")
ylim([-2e-4 2e-4])
legend("qs","qx","qy","qz")
grid on
hold off

end