function boat_animation(xk,uk,tk)

disp(tk)

real_time = toc;

disp(real_time)

if real_time > tk
    disp('Timestep too small')
end

while real_time < tk
    real_time = toc;
end

x = xk(1);
y = xk(2);
psi = xk(3);
dt = uk(1);
dr = uk(2);

cla;

xfuse_I = (([0 18 18 0]-9)./12)./3.28;
yfuse_I = (([0 0 7.75 7.75]-7.75/2)./12)./3.28;

cpsi = cos(psi);
spsi = sin(psi);
xfuse_B = cpsi*xfuse_I - spsi*yfuse_I;
yfuse_B = spsi*xfuse_I + cpsi*yfuse_I;

patch(xfuse_B+x,yfuse_B+y,[165 42 42]./255)

lr = (10/12)/3.28;
c = (2.25/12)/3.28;

boat_cg_I = [x;y];
r_cg_rudder_B = [-lr;0];
rFRONT_rudder_R = [c/2;0];
rREAR_rudder_R = [-2*c;0];

TIB = [cpsi -spsi; spsi cpsi];
TBR = [cos(dr) sin(dr);-sin(dr) cos(dr)];

rFRONT_I = boat_cg_I + TIB*r_cg_rudder_B + TIB*TBR*rFRONT_rudder_R;
rREAR_I = boat_cg_I + TIB*r_cg_rudder_B + TIB*TBR*rREAR_rudder_R;

hold on
plot([rFRONT_I(1) rREAR_I(1)],[rFRONT_I(2) rREAR_I(2)],'b-','LineWidth',3)

thrust_vec = boat_cg_I + TIB*[((18/12)/3.28)*dt/100 0]';
plot([boat_cg_I(1) thrust_vec(1)],[boat_cg_I(2) thrust_vec(2)],'r-')

title(tk)

world_lim = 2;
xlim([-world_lim+x world_lim+x])
ylim([-world_lim+y world_lim+y])
axis equal

drawnow





















