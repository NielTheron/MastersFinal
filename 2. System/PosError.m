yPos = x_true(1:3,:) - x_EKF(1:3,:);   % 3 x d
PosNorm = sqrt(sum(yPos.^2, 1));       % 1 x d vector of magnitudes
AvgPos = mean(PosNorm);
fprintf('Average Position Error: %.3f km\n', AvgPos);
