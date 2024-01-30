function val = blackbox(xk, tk, uk)

dt = 0.04;
time = 100; % enter desired time instant

%% Obtain instantaneous values
if tk == time
    der = Derivatives_old(xk, time, uk);
    % disp(der)
    val = der;
end
