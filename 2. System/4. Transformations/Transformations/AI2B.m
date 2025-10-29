%==========================================================================
% Niel Theron
% 19-06-2025
%==========================================================================
% The purpose of this function is to transform from the orbital reference
% frame to the body reference frame
%==========================================================================

function v_B = AI2B(v_I,q_I2B,r_I)

    % Get rotation matrix
    R_I2B = quat2rotm(q_I2B);
    %---
    
    % Make Transformation Matrix
    A_I2B = [R_I2B , -R_I2B*r_I;
            zeros(3,1), 1];
    %---

    % Multiply
    v_B = A_I2B*v_I;
    %--

end