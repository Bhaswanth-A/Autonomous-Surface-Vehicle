close all
clear 
clc

global uk WPctr

WPctr = 1;

%% Time vector
dt = 0.03;
time = 0:dt:200;

%% Initial conditions
xstate = zeros(6, length(time)); % x,y,psi,u,v,r
ystate = zeros(2, length(time));

x0 = [0 0 0 0 0 0]';

ANIMATE = 0;
AUTONOMOUS = 1;

if ANIMATE
    animation_setup();
    pause
end

xstate(:,1) = x0;

%% Range Kutta
for k = 1:length(time)-1
    xk = xstate(:,k);
    tk = time(k);

    %% Control
    if AUTONOMOUS
        uk = Controller(xk, tk);
    end

    %% Send current state to animation
    if ANIMATE
        boat_animation(xk,uk,tk);
    end

    %% Derivative calls
    k1 = Derivatives_old(xk,tk,uk);
    k2 = Derivatives_old(xk+k1*dt/2,tk+dt/2,uk);
    k3 = Derivatives_old(xk+k2*dt/2,tk+dt/2,uk);
    k4 = Derivatives_old(xk+k3*dt,tk+dt,uk);
    f = (1/6)*(k1+2*k2+2*k3+k4);

    xstate(:,k+1) = xstate(:,k) + f*dt;
    ustate(:,k+1) = uk;
end

%% Integration
% [T, R] = ode45(Kinetics,time,x0);


%% Extract states
x = xstate(1,:);
y = xstate(2,:);
psi = xstate(3,:);
u = xstate(4,:);
v = xstate(5,:);
r = xstate(6,:);
dt = ustate(1,:);
dr = ustate(2,:);

%% Plots

fig = figure;
plot(x,y,'b-','LineWidth',2)
hold on
xlabel('X (m)')
ylabel('Y (m)')

fig = figure;
plot(time,x,'b-','LineWidth',2)
hold on
xlabel('Time (sec)')
ylabel('X (m)')

figure()
hold on
plot(time,y)
xlabel('TIme (sec)')
ylabel('Y (m)')

figure()
hold on
plot(time, psi*180/pi)
xlabel('Time (sec)')
ylabel('\psi (deg)')

fig = figure();
plot(time,u,'b-','LineWidth',2)
hold on
xlabel('Time (sec)')
ylabel('U (m/s)')

fig = figure();
plot(time,v,'b-','LineWidth',2)
hold on
xlabel('Time (sec)')
ylabel('V (m/s)')

% fig = figure();
% plot(time,dt,'b-','LineWidth',2)
% hold on
% xlabel('Time (sec)')
% ylabel('Thrust (0-100%)')

fig = figure();
plot(time,dr*180/pi,'b-','LineWidth',2)
hold on
xlabel('Time (sec)')
ylabel('\delta_R (deg)')

dlmwrite('Boat_Data.txt',[time' xstate' ustate'])
















 
