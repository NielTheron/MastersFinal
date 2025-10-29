function catalogue_eci = LLA2ECI(catalogue_lla, we_p, t)
%========================================================================
% Convert catalogue from LLA to ECI coordinates
%========================================================================

numFeatures = size(catalogue_lla, 2);
catalogue_eci = zeros(3, numFeatures);

% Earth rotation angle
theta = we_p * t;

for i = 1:numFeatures
    if any(isnan(catalogue_lla(:,i)))
        catalogue_eci(:,i) = NaN(3,1);
        continue;
    end
    
    lat = catalogue_lla(1,i);
    lon = catalogue_lla(2,i);
    alt = catalogue_lla(3,i);
    
    % Convert LLA to ECEF
    [x_ecef, y_ecef, z_ecef] = geodetic2ecef(wgs84Ellipsoid('km'), lat, lon, alt);
    
    % Rotate ECEF to ECI (reverse of ECI to ECEF)
    R_ECEF2ECI = [cos(theta)   sin(theta)   0;
                  -sin(theta)  cos(theta)   0;
                   0           0            1];
    
    P_eci = R_ECEF2ECI * [x_ecef; y_ecef; z_ecef];
    catalogue_eci(:,i) = P_eci;
end

end