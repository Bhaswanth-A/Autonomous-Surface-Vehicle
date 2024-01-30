function uk = Controller(xk, tk)

global WPctr Pen i
 
%% Extract states
x = xk(1);
y = xk(2);
psi = xk(3);
u = xk(4);
v = xk(5);
r = xk(6);

% Compute Control - commands throttle
uc = 5; % Enter desired speed
kp = 200;
du = kp*(uc-u);

if du > 100
    du = 100;
end
if du < 0
    du = 0;
end

if u > uc
    Pen = Pen + 1;
end

%% Rudder Angle command
kpsi_p = 2.5;
kpsi_d = 1.25;
yc = [0 0 80 80]*3; % waypoints
xc = [0 80 80 0]*3; % waypoints
yci = yc(WPctr);
xci = xc(WPctr);

psic = atan2(yc(WPctr)-y,xc(WPctr)-x);
del_psi = -atan2(sin(psi)*cos(psic)-cos(psi)*sin(psic), cos(psi)*cos(psic)+sin(psi)*sin(psic));

if rad2deg(abs(del_psi)) > 5
    if i < 85
        Pen = Pen + 0.1 * abs(del_psi);
    end
    i = i + 1;
    
end

disp(Pen)

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

uk = [du;dr]; % Outputs throttle and rudder command






















