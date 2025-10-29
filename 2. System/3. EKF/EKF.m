%==========================================================================
% Niel Theron
% 20-03-2025
%==========================================================================
% This is the Main EKF function
% The purpose of this function is to estimate the state vector of the
% system
%==========================================================================
% y_est         : Estimated measurement from the estimated state vector
% x_est         : Estimated state
% P_est         : Co-varaince matrix for the extimated state)
% n_z           : Number of measured states
% n_x           : Number of states in the statevector
% K_est         : Kalman Gain
% c             : Internal catalogue vector.
%==========================================================================
function [zhat_ET, x_EKF, P_EKF, K_ET] = EKF( ...
 cat,x_EKF,P_EKF,I_f,Q_f,dt_f,Mu_f,Re_f, J2_f, we_f, t, ...
 z_ET, z_ST, z_GPS, z_GYR, z_MAG, z_CSS, ...
 R_ET, R_ST, R_GPS, R_GYR, R_MAG, R_CSS)

%% Initialise Varaibles

n_x = 13;
n_z = 3;
n_f = size(z_ET,2);
K_ET = zeros(n_x,n_z,n_f);
zhat_ET = zeros(n_z,n_f);

%% Prediction step
xp_EKF = StatePredictionF(x_EKF,I_f,dt_f,Mu_f,Re_f,J2_f);
if xp_EKF(7) < 0  % assuming [w,x,y,z] format
    xp_EKF(7:10) = -xp_EKF(7:10);
end
Pp_EKF = CovaraincePrediction(xp_EKF,P_EKF,Q_f,dt_f,I_f,Mu_f,Re_f,J2_f);
%---

%% Update step

% GPS
if norm(z_GPS) ~= 0
    zhat_GPS    = H_GPS_function(xp_EKF,we_f,t);
    H_GPS       = H_GPS_jacobian(xp_EKF,we_f,t);

    K_GPS       = GainUpdate(H_GPS,Pp_EKF,R_GPS);
    xp_EKF      = StateUpdate(xp_EKF,K_GPS,z_GPS,zhat_GPS);
    Pp_EKF      = CovarianceUpdate(K_GPS,Pp_EKF,R_GPS,H_GPS);
end
%---

% Gyroscope
if norm(z_GYR) ~= 0
    zhat_GYR    = H_GYR_function(xp_EKF,Mu_f);
    H_GYR       = H_GYR_jacobian(xp_EKF,Mu_f);

    K_GYR       = GainUpdate(H_GYR,Pp_EKF,R_GYR);
    xp_EKF      = StateUpdate(xp_EKF,K_GYR,z_GYR, zhat_GYR);
    Pp_EKF      = CovarianceUpdate(K_GYR,Pp_EKF,R_GYR,H_GYR);
end
%---

% Earth Tracker
for i = 1:n_f
    if norm(z_ET(:,i)) ~= 0
        zhat_ET(:,i)        = H_ET_function(xp_EKF,cat(:,i),we_f,t);
        H_ET                = H_ET_jacobian(xp_EKF,cat(:,i),we_f,t);

        K_ET(:,:,i) = GainUpdate(H_ET,Pp_EKF,R_ET);
        xp_EKF      = StateUpdate(xp_EKF,K_ET(:,:,i),z_ET(:,i),zhat_ET(:,i));
        Pp_EKF      = CovarianceUpdate(K_ET(:,:,i),Pp_EKF,R_ET,H_ET);

        xp_EKF(7:10) = quatnormalize(xp_EKF(7:10).');

    end
end
%---

% Star Tracker
for i = 1:20
    if norm(z_ST(:,i)) ~= 0
        zhat_i  = H_ST_function_i(xp_EKF, i);
        H_i     = H_ST_jacobian_i(xp_EKF, i);

        z_i     = z_ST(:,i);                  
        K_i     = GainUpdate(H_i, Pp_EKF, R_ST);  
        xp_EKF  = StateUpdate(xp_EKF, K_i, z_i, zhat_i);
        Pp_EKF  = CovarianceUpdate(K_i, Pp_EKF, R_ST, H_i);
    end
end
%---

% Magnetometer
if norm(z_MAG) ~= 0
    zhat_MAG    = H_MAG_function(xp_EKF,we_f,t);    
    H_MAG       = H_MAG_jacobian(xp_EKF,we_f,t);

    K_MAG       = GainUpdate(H_MAG, Pp_EKF, R_MAG);
    xp_EKF      = StateUpdate(xp_EKF, K_MAG, z_MAG, zhat_MAG);
    Pp_EKF      = CovarianceUpdate(K_MAG, Pp_EKF, R_MAG, H_MAG);
end
%---

% CSS
if norm(z_CSS) ~= 0
  
    zhat_CSS    = H_CSS_function(xp_EKF);
    H_CSS       = H_CSS_jacobian(xp_EKF);

    K_CSS       = GainUpdate(H_CSS, Pp_EKF, R_CSS);
    xp_EKF      = StateUpdate(xp_EKF, K_CSS, z_CSS, zhat_CSS);
    Pp_EKF      = CovarianceUpdate(K_CSS, Pp_EKF, R_CSS, H_CSS);
%---

end

%% Pass the variables
xp_EKF(7:10) = quatnormalize(xp_EKF(7:10).');
x_EKF = xp_EKF;
if x_EKF(7) < 0  % assuming [w,x,y,z] format
    x_EKF(7:10) = -x_EKF(7:10);
end
P_EKF = Pp_EKF;
%---

end

