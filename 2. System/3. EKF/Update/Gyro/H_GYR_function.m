%==========================================================================
% Niel Theron
% 02-10-2025
%==========================================================================
% The purpose of this function is to estimate the gyroscope measurement
% from the estimate state
%==========================================================================

function zhat_GYR = H_GYR_function(x_est, Mu)

% Calculate WO/I (orbital angular velocity in orbital frame)
w_I2O_in_O = sqrt(Mu/(norm(x_est(1:3))^3)) * [0; -1; 0];

% Extract individual quaternion components for clarity
qs = x_est(7);  % Scalar component
qx = x_est(8);  % X vector component
qy = x_est(9);  % Y vector component
qz = x_est(10);  % Z vector component

% Construct Rotation Matrix from ECI to Body Frame
R_O2B = [
    1 - 2*(qy^2 + qz^2),  2*(qx*qy - qs*qz),  2*(qx*qz + qs*qy);
    2*(qx*qy + qs*qz),   1 - 2*(qx^2 + qz^2),  2*(qy*qz - qs*qx);
    2*(qx*qz - qs*qy),   2*(qy*qz + qs*qx),   1 - 2*(qx^2 + qy^2)
    ];

% Get Total Angular Velocity in Body Frame
w_I2O_in_B = R_O2B * w_I2O_in_O;
w_I2B_in_B = w_I2O_in_B + x_est(11:13);


% Final measurement
zhat_GYR = w_I2B_in_B;

end
