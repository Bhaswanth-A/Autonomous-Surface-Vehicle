function dxdt = Derivatives_old(xstate,t,ustate)

%% Extract states
x = xstate(1);
y = xstate(2);
psi = xstate(3);
u = xstate(4);
v = xstate(5);
r = xstate(6);
du = ustate(1);
dr = ustate(2);

flowDir = 45;
flowVel = 0.2;

dAngle = flowDir - psi;
if dAngle > 2*pi
    dAngle = dAngle - 2*pi;
elseif dAngle < 0
    dAngle = dAngle + 2*pi;
end

flowu = flowVel * cos(dAngle);
flowv = flowVel * sin(dAngle);

load("parameters.mat")

u_dot = (auv.Xuu*u*abs(u)+(auv.Xvr+auv.m)*v*r+(auv.Xrr+auv.m*auv.xg)*r*r+du);
v_dot = (auv.Yvv*v*abs(v)+auv.Yrr*r*abs(r)+(auv.Yur-auv.m)*u*r+auv.Yuv*u*v+auv.Yuudelta*u*u*dr);
r_dot = (auv.Nvv*v*abs(v)+auv.Nrr*r*abs(r)+(auv.Nur-auv.m*auv.xg)*u*r+auv.Nuv*u*v+auv.Nuudelta*u*u*dr);

dedt = auv.M\[u_dot v_dot r_dot]';

u = u + flowu;
v = v + flowv;

x_dot = u*cos(psi) - v*sin(psi);
y_dot = u*sin(psi) + v*cos(psi);

psi_dot = r;

dxdt = [x_dot y_dot psi_dot dedt(1) dedt(2) dedt(3)]';

