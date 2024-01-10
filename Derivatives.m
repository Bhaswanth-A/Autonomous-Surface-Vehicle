function dxdt = Derivatives(xstate,t,ustate)

%% Extract states
x = xstate(1);
y = xstate(2);
psi = xstate(3);
u = xstate(4);
v = xstate(5);
r = xstate(6);
ur = ustate(1);
rr = ustate(2);

A1 = -0.0152;
A2 = 0.1305;
A3 = 0.0;
A4 = 0.0508;
A5 = 0.6245;
A6 = -0.0075;
A7 = 0.1831;
A8 = -0.0111;
A9 = 0.0139;
A10 = 0.0194;
A11 = 0.0505;
A12 = 0.0268;
A13 = -0.4451;
A14 = 0.7005;
A15 = 106.4701;
A16 = 0.0385;

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

% Linearization
K_efficiency = 0.8;
L_rudder = 6.75 * 10^(-3); %m2
LG = 0.673; %m

B1 = rr + A10*abs(v)*v + A11*abs(r)*v + A12*abs(v)*r + A13*abs(r)*r - A16*u*v;
disp(B1)
K_denom = A15 * K_efficiency * L_rudder * u * LG;
theta_ref = asin(B1/K_denom);

%% Rudder Model
tau_rudder = 0.0986; % seconds

num = [1];
denom = [tau_rudder 1];
theta_tf = tf(num, denom);

theta_rudder = theta_ref * theta_tf;

F_rudder = K_efficiency * sin(theta_rudder) * L_rudder * u;
tau_r = F_rudder * LG; % output

%% Thruster Model
% m = 8.4367; % kg (mass of USV)
B = 3.10636; % linear damper
K_thrust = 0.020188;

rho = 1025; % kg/m3 density of water
D = 0.05; % m propeller diameter
% n = ;

% tau_u = K_thrust * rho * D^4 * n^2 - F_rudder;
tau_u = B * u - F_rudder;


%% Dynamics

u_dot = A1*v*r - A2*u - A3*abs(u)*u + A4*tau_u;
v_dot = A5*u*r -A6*v -A7*abs(v)*v - A8*abs(r)*v - A9*abs(v)*r;
r_dot = -A10*abs(v)*v - A11*abs(r)*v - A12*abs(v)*r - A13*abs(r)*r - A14*r + A15*tau_r + A16*u*v;

u_new = u + flowu;
v_new = v + flowv;

x_dot = u_new*cos(psi) - v_new*sin(psi);
y_dot = u_new*sin(psi) + v_new*cos(psi);

psi_dot = r;

dxdt = [x_dot y_dot psi_dot u_dot v_dot r_dot]';








