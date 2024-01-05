uk = [0;0];
fig = figure('Name','Boat_Video_Game','keypressfcn',@keypress,'KeyReleaseFcn',@keyrelease);
xlabel('X (m)')
ylabel('Y (m)')
grid on
tic
set(gca,'YDir','reverse')
