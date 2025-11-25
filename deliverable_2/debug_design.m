% Debug a single design point
% Add official functions LAST so they take precedence
addpath('../ee_functions');
addpath('../Project3_Functions');

% Fixed parameters
rho_composite = 1600;
rho_steel = 7850;
rho_magnet = 7850;
max_pm_tip_speed = 175;
max_steel_tip_speed = 175;
max_composite_tip_speed = 900;
axial_clearance = 0.020;
epsilon = 0.8;
sigma = 5.67e-8;
T_amb = 293;

% Design variables - baseline-like
t_mag = 0.006;
omega_max_rpm = 40000;
omega_max = omega_max_rpm * 2*pi / 60;
omega_min = omega_max / 2;

fprintf('Design variables:\n');
fprintf('  Magnet thickness: %.1f mm\n', t_mag*1000);
fprintf('  Max speed: %.0f RPM\n', omega_max_rpm);

% Step 1: Shaft diameter
r_shaft_max = (max_pm_tip_speed / omega_max) - t_mag;
r_shaft_steel = max_steel_tip_speed / omega_max;
r_shaft = min(r_shaft_max, r_shaft_steel);
d_shaft = 2 * r_shaft;

fprintf('\nSizing:\n');
fprintf('  Shaft diameter: %.1f mm (limit: %.1f mm from PM, %.1f mm from steel)\n', ...
    d_shaft*1000, 2*r_shaft_max*1000, 2*r_shaft_steel*1000);

% Step 2: Flywheel diameter
r_flywheel = max_composite_tip_speed / omega_max;
d_flywheel = 2 * r_flywheel;
fprintf('  Flywheel diameter: %.1f mm\n', d_flywheel*1000);

% Step 3: Motor length
I_rated_pu = 0.8;
shear = magneticShear(t_mag, I_rated_pu);
fprintf('  Magnetic shear: %.1f Pa at %.1f pu current\n', shear, I_rated_pu);

P_target = 150000;
torque_per_length = shear * pi * d_shaft * (d_shaft/2);
L_motor_min = P_target / (torque_per_length * omega_max);
L_motor = max(0.100, min(0.400, ceil(L_motor_min * 100) / 100));

motor_area = pi * d_shaft * L_motor;
P_rated = shear * motor_area * (d_shaft/2) * omega_max;
fprintf('  Motor length: %.1f mm (min needed: %.1f mm)\n', L_motor*1000, L_motor_min*1000);
fprintf('  Rated power: %.1f kW\n', P_rated/1000);

% Step 4: Flywheel length
r_outer = d_flywheel / 2;
r_inner = d_shaft / 2;
E_target = 18e3 * 3600;  % 18 kWh
V_per_length = pi * (r_outer^2 - r_inner^2);
m_per_length = rho_composite * V_per_length;
I_per_length = 0.5 * m_per_length * (r_outer^2 + r_inner^2);
I_required = 2 * E_target / (omega_max^2 - omega_min^2);
L_flywheel_min = I_required / I_per_length;
L_flywheel = max(0.300, min(2.000, ceil(L_flywheel_min * 10) / 10));
fprintf('  Flywheel length: %.1f mm (min needed: %.1f mm)\n', L_flywheel*1000, L_flywheel_min*1000);

% Calculate mass and inertia
V_flywheel = pi * (r_outer^2 - r_inner^2) * L_flywheel;
m_flywheel = rho_composite * V_flywheel;
L_shaft = L_flywheel + L_motor + 4*axial_clearance + 0.3;
V_shaft = pi * r_inner^2 * L_shaft;
m_shaft = rho_steel * V_shaft;
r_mag = r_inner + t_mag;
V_magnet = pi * (r_mag^2 - r_inner^2) * L_motor;
m_magnet = rho_magnet * V_magnet;
m_amb = 0.10 * m_shaft;
m_total = m_flywheel + m_shaft + m_magnet + m_amb;

I_flywheel = 0.5 * m_flywheel * (r_outer^2 + r_inner^2);
I_shaft = 0.5 * m_shaft * r_inner^2;
I_magnet = 0.5 * m_magnet * (r_mag^2 + r_inner^2);
I_total = I_flywheel + I_shaft + I_magnet;

fprintf('\nMass and Inertia:\n');
fprintf('  Total mass: %.1f kg\n', m_total);
fprintf('  Total inertia: %.4f kg.m^2\n', I_total);

E_max = 0.5 * I_total * omega_max^2;
E_min = 0.5 * I_total * omega_min^2;
E_stored = E_max - E_min;
fprintf('  Energy capacity: %.2f kWh\n', E_stored/3.6e6);
fprintf('  Specific power: %.3f kW/kg\n', P_rated/1000/m_total);
fprintf('  Specific energy: %.2f Wh/kg\n', E_stored/3600/m_total);

% Simulate cycle
fprintf('\nSimulating cycle...\n');
dt = 30.0;
t_cycle = 0:dt:21600;
omega_current = (omega_max + omega_min) / 2;
min_soc = 50;
max_temp = 20;
E_in = 0;
E_out = 0;
A_surface = 2*pi*r_outer*L_flywheel + 2*pi*r_outer^2;

for k = 1:length(t_cycle)
    P_grid = newStorageCycle(t_cycle(k));
    omega_rpm = omega_current * 60 / (2*pi);
    SoC = 100 * (omega_current - omega_min) / (omega_max - omega_min);

    if SoC < min_soc
        min_soc = SoC;
    end

    I_pu = I_rated_pu * abs(P_grid) / P_rated;
    I_pu = min(I_pu, 1.0);

    P_rotor = rotorLosses(t_mag, d_shaft, L_motor, I_pu, omega_rpm);
    P_stator = statorLosses(t_mag, d_shaft, L_motor, I_pu, omega_rpm);
    P_loss = P_rotor + P_stator;

    T_rotor = (T_amb^4 + P_rotor/(epsilon*sigma*A_surface))^(1/4);
    T_celsius = T_rotor - 273;
    if T_celsius > max_temp
        max_temp = T_celsius;
    end

    E_current = 0.5 * I_total * omega_current^2;
    P_net = P_grid + P_loss;
    dE = -P_net * dt;
    E_current = E_current + dE;

    if E_current > 0
        omega_current = sqrt(2 * E_current / I_total);
    end
    omega_current = max(omega_min*0.9, min(omega_max*1.05, omega_current));

    if P_grid > 0
        E_out = E_out + P_grid * dt;
    else
        E_in = E_in + abs(P_grid) * dt;
    end
end

SoC_final = 100 * (omega_current - omega_min) / (omega_max - omega_min);
E_recovery = max(0, 0.5 * I_total * ((omega_max+omega_min)/2)^2 - 0.5 * I_total * omega_current^2);
if (E_in + E_recovery) > 0
    efficiency = (E_out / (E_in + E_recovery)) * 100;
else
    efficiency = 0;
end

fprintf('\nCycle Results:\n');
fprintf('  Min SoC: %.1f%%\n', min_soc);
fprintf('  Max temperature: %.1f C\n', max_temp);
fprintf('  Energy in: %.2f kWh\n', E_in/3.6e6);
fprintf('  Energy out: %.2f kWh\n', E_out/3.6e6);
fprintf('  Recovery needed: %.2f kWh\n', E_recovery/3.6e6);
fprintf('  Efficiency: %.1f%%\n', efficiency);

fprintf('\nViability check:\n');
fprintf('  Temp < 100C: %s\n', iif(max_temp < 100, 'PASS', 'FAIL'));
fprintf('  SoC > 0: %s\n', iif(min_soc > 0, 'PASS', 'FAIL'));
fprintf('  Eff 70-105: %s\n', iif(efficiency > 70 && efficiency < 105, 'PASS', 'FAIL'));
fprintf('  Power > 60kW: %s\n', iif(P_rated > 60000, 'PASS', 'FAIL'));

function r = iif(cond, a, b)
    if cond
        r = a;
    else
        r = b;
    end
end
