%==========================================================================
% Niel Theorn
% 19-06-2025
%==========================================================================
% This function is used to convert from the ECR reference frame to the ECI
% reference frame
%=========================================================================
% Input:
%   v_R : ECR Vector [km] [3x1]
%   t   : Time [s]
%   we  : Angular veloctiy if the Earth [rad/s]
% Output:
%   v_I : ECI Vector [km] [3x1]
% Variables:
%   theta : Rotation angle [rad]
%==========================================================================
function v_I = AR2I(v_R,we,t)

    % Make Rotation matrix
    theta = -we*t;
    R_E2I =  [cos(theta) -sin(theta) 0;
             sin(theta) cos(theta) 0;
             0           0          1];
    %---

    % Multiply
    v_I = R_E2I*v_R;
    %---

end

