% Debug script for storage cycle
addpath('../Project3_Functions');
addpath('../ee_functions');

% Test the storage cycle
t = 0:10:21600;
p = newStorageCycle(t);
fprintf('Storage cycle stats:\n');
fprintf('  Max power: %.2f kW\n', max(p)/1000);
fprintf('  Min power: %.2f kW\n', min(p)/1000);
fprintf('  Mean power: %.2f kW\n', mean(p)/1000);
fprintf('  Net energy discharged: %.2f kWh\n', sum(p(p>0))*10/3.6e6);
fprintf('  Net energy charged: %.2f kWh\n', sum(abs(p(p<0)))*10/3.6e6);

% Test a design similar to baseline
t_mag = 0.006;
omega_max_rpm = 40000;
omega_max = omega_max_rpm * 2*pi/60;
omega_min = omega_max/2;

% Size like baseline
d_shaft = 0.084;
d_flywheel = 0.430;
L_flywheel = 1.0;
L_motor = 0.25;

% Calculate inertia
rho_composite = 1600;
rho_steel = 7850;
r_outer = d_flywheel/2;
r_inner = d_shaft/2;
V_fw = pi*(r_outer^2-r_inner^2)*L_flywheel;
m_fw = rho_composite * V_fw;
I_total = 0.5*m_fw*(r_outer^2+r_inner^2);

% Energy stored
E_stored = 0.5*I_total*(omega_max^2-omega_min^2);
fprintf('\nBaseline-like design:\n');
fprintf('  Energy capacity: %.2f kWh\n', E_stored/3.6e6);

% Simulate at 50% SoC
omega = (omega_max+omega_min)/2;
E_current = 0.5*I_total*omega^2;
E_at_0_SoC = 0.5*I_total*omega_min^2;
available = E_current - E_at_0_SoC;
fprintf('  Available energy at 50%% SoC: %.2f kWh\n', available/3.6e6);
fprintf('  Energy at 100%% SoC: %.2f kWh\n', 0.5*I_total*omega_max^2/3.6e6);
fprintf('  Energy at 0%% SoC: %.2f kWh\n', E_at_0_SoC/3.6e6);
