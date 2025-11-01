q_true = x_true(7:10, :);  % 4 x N
q_EKF  = x_EKF(7:10, :);   % 4 x N
nSteps = size(q_true, 2);

angle_deg = zeros(1, nSteps);
v_ref = [1; 0; 0];

for k = 1:nSteps
    q_t = q_true(:, k)';   % 1x4
    q_e = q_EKF(:, k)';    % 1x4
    
    q_e_conj = [q_e(1), -q_e(2:4)];      % conjugate
    dqk = quatmultiply(q_t, q_e_conj);   % error quaternion
    dqk = dqk / norm(dqk);               % normalize
    
    % Rotate reference vector using error quaternion
    v_rot = quatrotate(dqk, v_ref');     
    v_rot = v_rot';                      
    
    % Directional error
    cos_theta = dot(v_ref, v_rot) / (norm(v_ref) * norm(v_rot));
    cos_theta = max(min(cos_theta, 1), -1);
    angle_deg(k) = rad2deg(acos(cos_theta));
end

% Plot
figure;
plot(time,angle_deg);
xlabel('Time Step');
ylabel('Directional Error (deg)');
title('Direction Error Over Time');
grid on;

avg_error = mean(angle_deg);
max_error = max(angle_deg);
min_error = min(angle_deg);

fprintf('Average Direction Error: %.3f deg\n', avg_error);
fprintf('Maximum Direction Error: %.3f deg\n', max_error);
fprintf('Minimum Direction Error: %.3f deg\n', min_error);