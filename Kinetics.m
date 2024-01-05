function dxdt = Kinetics(xstate,t,ustate)

%% Extract states
x = xstate(1);
y = xstate(2);
psi = xstate(3);
u = xstate(4);
v = xstate(5);
r = xstate(6);
dt = ustate(1);
Deltar = ustate(2);

XProp = 3.86;
NDisturb = 0;

global auv;

u_dot = auv.Xuu*u*abs(u)+(auv.Xvr+auv.m)*v*r+(auv.Xrr+auv.m*auv.xg)*r*r+Xprop;
v_dot = auv.Yvv*v*abs(v)+auv.Yrr*r*abs(r)+(auv.Yur-auv.m)*u*r+auv.Yuv*u*v+auv.Yuudelta*u*u*Deltar;
r_dot = NDisturb+auv.Nvv*v*abs(v)+auv.Nrr*r*abs(r)+(auv.Nur-auv.m*auv.xg)*u*r+auv.Nuv*u*v+auv.Nuudelta*u*u*Deltar;

dxdt = auv.M\[u_dot;v_dot;r_dot];