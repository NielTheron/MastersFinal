function R_I2O = RI2O(r_I,v_I)

    % Define orbital (LVLH) frame vectors in ECI coordinates
    z_O = -r_I / norm(r_I);              % Nadir (toward Earth)
    y_O = cross(v_I, r_I);               % Cross-track (normal to orbital plane)
    y_O = y_O / norm(y_O);               % Normalize
    x_O = cross(y_O, z_O);               % Along-track (completes right-handed system)
    %---

    % Create Rotation Matrix
    R_I2O = [x_O.'; y_O.'; z_O.'];
    %---

end

