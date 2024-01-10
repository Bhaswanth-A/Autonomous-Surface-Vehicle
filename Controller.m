function uk = Controller(xk, tk)

global WPctr
 
%% Extract states
x = xk(1);
y = xk(2);
psi = xk(3);
u = xk(4);
v = xk(5);
r = xk(6);

%% Compute Control - commands throttle
% uc = 1.5; % 5m/s before
% kp = 70;
% 
% dt = kp*(uc-u);
% if dt > 100
%     dt = 100;
% end
% if dt < 0
%     dt = 0;
% end

dt = 0;

%% Rudder Angle command
kpsi_p = 2.5;
kpsi_d = 1.25;
yc = [0 0 80 80]; % waypoints
xc = [0 80 80 0]; % waypoints
yci = yc(WPctr);
xci = xc(WPctr);

psic = atan2(yc(WPctr)-y,xc(WPctr)-x);
del_psi = -atan2(sin(psi)*cos(psic)-cos(psi)*sin(psic), cos(psi)*cos(psic)+sin(psi)*sin(psic));

% If boat gets within 10m of waypoint, switch to next waypoint
if sqrt((x-xc(WPctr))^2 + (y-yc(WPctr))^2) < 5
    WPctr = WPctr + 1;
    if WPctr > length(xc)
        WPctr = 1;
    end
end

rc = 0;
dr = kpsi_p*del_psi + kpsi_d*(rc-r);
if dr > 30*pi/180
    dr = 30*pi/180;
end
if dr < -30*pi/180
    dr = -30*pi/180;
end

uk = [dt;dr]; % Outputs throttle and rudder command






















