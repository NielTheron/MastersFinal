
% Initialize variables
position = x_input; % Assuming 2D position

GPS_noise = 0.1; % Example noise value

GPS_drift_prev = 0; % Initial drift

drift_rate = 0.01; % Example drift rate

we_p = 0; % Placeholder for additional parameter

t = 0; % Initial time
dt_p = 1; % Time step


for r = 1:4199

    z_GPS = GPS(position, GPS_noise, GPS_drift_prev, drift_rate, we_p, t, dt_p)
    z_hat

end
