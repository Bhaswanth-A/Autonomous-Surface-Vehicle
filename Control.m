function uk = Control(xk, tk)

global WPctr
 
%% Extract states
x = xk(1);
y = xk(2);
psi = xk(3);
u = xk(4);
v = xk(5);
r = xk(6);

% %% Rudder
% tau_rudder = 0;
% theta_ref = ;
% 
% K_efficiency = 0.8;
% L_rudder = 6.75 * 10^(-3); %m2
% LG = 0.673; %m
% 
% num = [1];
% denom = [tau_rudder 1];
% theta_tf = tf(num, denom);
% 
% theta_rudder = theta_ref * theta_tf;
% 
% F_rudder = K_efficiency * sin(theta_rudder) * L_rudder * u;
% tau_r = F_rudder * LG; % output
% 
% %% Thruster Engine
% m = 8.4367; % kg (mass of USV)
% B = 3.10636; % linear damper
% K_thrust = 0.020188;
% 
% rho = 1025; % kg/m3 density of water
% D = 0.05; % m propeller diameter
% n = ;
% 
% tau_u = K_thrust * rho * D^4 * n^2 - F_rudder;

%% Compute velocity
u_ref = 1.5; % m/s
kp = 0.5;
ki = 0.0653;
kd = 0;

ur = pid(kp,ki,kd);

%% Rudder Angle command
kpsip = 2.5;
kpsid = 1.25;
yc = [0 0 20 20]*2; % waypoints
xc = [0 20 20 0]*2; % waypoints
yci = yc(WPctr);
xci = xc(WPctr);

psic = atan2(yc(WPctr)-y,xc(WPctr)-x);
delpsi = -atan2(sin(psi)*cos(psic)-cos(psi)*sin(psic), cos(psi)*cos(psic)+sin(psi)*sin(psic));

% If boat gets within 10m of waypoint, switch to next waypoint
if sqrt((x-xc(WPctr))^2 + (y-yc(WPctr))^2) < 10
    WPctr = WPctr + 1;
    if WPctr > length(xc)
        WPctr = 1;
    end
end

rc = 0;
rr = kpsip*delpsi + kpsid*(rc-r);
if rr > 30*pi/180
    rr = 30*pi/180;
end
if rr < -30*pi/180
    rr = -30*pi/180;
end

uk = [ur;rr];










