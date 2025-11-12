%==========================================================================
% Niel Theron
% 20-03-2025
%==========================================================================
% The Purpose of this function is determine the next state in the system
% using non-linear equations
% I         : Moment of Inertia
% Mu        : The gravitational constant of the Earth
% R         : Magnitude of the distance vector
% x = xn    : Current States
% f         : Output of the F_function
%==========================================================================
function f = FFunctionF(x,I,dt,Mu,Re,J2)

% Initialise Varaibles
f = zeros(13,1);
%---

% Calcualte the gravitaional effect
R = norm(x(1:3));
C = -Mu/R^3;
%---

% Work out the effect of J2
z2 = x(3)^2;
r2 = R^2;
J2_factor = 1.5 * J2 * Mu * Re^2 / R^5;
ax_J2 = J2_factor * x(1) * (5*z2/r2 - 1);
ay_J2 = J2_factor * x(2) * (5*z2/r2 - 1);
az_J2 = J2_factor * x(3) * (5*z2/r2 - 3);
%---

% Small angular velocity bias (rad/s)
bias = [0.000000001,0,0];%[0.0000005; 0.000003; -0.000002];  % small constant bias for wx, wy, wz

% Add bias to angular velocity before quaternion integration
wx = x(11) + bias(1);
wy = x(12) + bias(2);
wz = x(13) + bias(3);



% Normalise Quaternion
x(7:10) = quatnormalize(x(7:10).');
%---
% Caculate the next State
f(1,1)  = x(1) + x(4)*dt;                                         % rx
f(2,1)  = x(2) + x(5)*dt;                                         % ry
f(3,1)  = x(3) + x(6)*dt;                                         % rz
f(4,1)  = x(4) + (C*x(1) + ax_J2)*dt;                             % vx
f(5,1)  = x(5) + (C*x(2) + ay_J2)*dt;                             % vy
f(6,1)  = x(6) + (C*x(3) + az_J2)*dt;                             % vz


f(7,1)  = x(7) + 0.5*(-wx*x(8)-wy*x(9)-x(13)*x(10))*dt;     % qs
f(8,1)  = x(8) + 0.5*( wx*x(7)+wz*x(9)-wy*x(10))*dt;     % qx
f(9,1)  = x(9) + 0.5*( wy*x(7)-wz*x(8)+wx*x(10))*dt;     % qy
f(10,1) = x(10)+ 0.5*( wz*x(7)+wy*x(8)-wx*x(9))*dt;      % qz
f(11,1) = wx   + ((I(2)-I(3))*x(12)*x(13))/I(1)*dt;               % wx
f(12,1) = wy   + ((I(3)-I(1))*x(13)*x(11))/I(2)*dt;               % wy
f(13,1) = wz   + ((I(1)-I(2))*x(11)*x(12))/I(3)*dt;               % wz
%---

theta_offset = deg2rad(0.0007);
q_offset = [cos(theta_offset/2); 0; 0; sin(theta_offset/2)];  % rotation about z

% Multiply quaternions: q_new = q_offset * q_current
f(7:10) = quatmultiply(q_offset', f(7:10)');
f(7:10) = quatnormalize(f(7:10).');  % normalize after adding offset



f(7:10) = quatnormalize(f(7:10).');

% Ensure same format
if f(7) < 0  % assuming [w,x,y,z] format
    f(7:10) = -f(7:10);
end
%---