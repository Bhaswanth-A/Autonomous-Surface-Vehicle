function dxdt = Derivatives(xstate,t,ustate)

%% Extract states
x = xstate(1);
y = xstate(2);
psi = xstate(3);
u = xstate(4);
v = xstate(5);
r = xstate(6);
dt = ustate(1);
Deltar = ustate(2);

Xprop = 3.86;
NDisturb = 0;

flowDir = 0;
flowVel = 0;

dAngle = flowDir - psi;
if dAngle > 2*pi
    dAngle = dAngle - 2*pi;
elseif dAngle < 0
    dAngle = dAngle + 2*pi;
end

flowu = flowVel * cos(dAngle);
flowv = flowVel * sin(dAngle);

global auv;

auv.m = 30.51; %kg
auv.Izz = 3.45;

auv.xg = 0; %m
auv.zg = 0.0196;

auv.Xdotu = -0.93;
auv.Xuu = -1.62; %kg/m
auv.Xvr = 35.5;
auv.Xrr = -1.93;

auv.Ydotv = -35.5;
auv.Ydotr = 1.93;
auv.Yvv = -131;
auv.Yrr = 0.632;
auv.Yur = 5.22;
auv.Yuv = -28.6;
auv.Yuudelta = -9.64;

auv.Ndotv = 1.93;
auv.Ndotr = -4.88;
auv.Nvv = -3.18;
auv.Nrr = -9.4;
auv.Nur = -2;
auv.Nuv = -2.4;
auv.Nuudelta = 6.15;

auv.M = [auv.m-auv.Xdotu,0,0;0,auv.m-auv.Ydotv,auv.m*auv.xg-auv.Ydotr;0,auv.m*auv.xg-auv.Ndotv,auv.Izz-auv.Ndotr];

u_dot = auv.Xuu*u*abs(u)+(auv.Xvr+auv.m)*v*r+(auv.Xrr+auv.m*auv.xg)*r*r+Xprop;
v_dot = auv.Yvv*v*abs(v)+auv.Yrr*r*abs(r)+(auv.Yur-auv.m)*u*r+auv.Yuv*u*v+auv.Yuudelta*u*u*Deltar;
r_dot = NDisturb+auv.Nvv*v*abs(v)+auv.Nrr*r*abs(r)+(auv.Nur-auv.m*auv.xg)*u*r+auv.Nuv*u*v+auv.Nuudelta*u*u*Deltar;

dedt = auv.M\[u_dot v_dot r_dot]';

u = u + flowu;
v = v + flowv;

x_dot = u*cos(psi) - v*sin(psi);
y_dot = u*sin(psi) + v*cos(psi);

psi_dot = r;

dxdt = [x_dot y_dot psi_dot dedt(1) dedt(2) dedt(3)]';

