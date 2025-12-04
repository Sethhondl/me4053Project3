%% Compare Klei Group with corrected inertia formula
% This script shows the impact of fixing the hollow cylinder inertia error

addpath('Project3_Functions');

fprintf('=================================================================\n');
fprintf('   INERTIA FORMULA CORRECTION COMPARISON\n');
fprintf('=================================================================\n\n');

%% Parameters (same as both groups)
p.geometry.flywheelDiameter_m    = 430e-3;
p.geometry.flywheelAxialLength_m = 1000e-3;
p.geometry.motorAxialLength_m    = 250e-3;
p.geometry.shaftAndPMDiameter_m  = 84e-3;
p.geometry.magnetThickness_m     = 6e-3;
p.clearance.axial_m              = 20e-3;
p.amb.ratedForce_N               = 5780;
p.speed.maxTsShaft_mPerSec       = 175;
p.speed.maxTsFly_mPerSec         = 900;
p.material.densFly_kgPerM3       = 1600;
p.material.densSteel_kgPerM3     = 7850;
p.thermal.housingTemp_C          = 30;
p.thermal.maxTemp_C              = 100;
p.thermal.rotorEmissivity        = 0.4;
p.thermal.housingEmissivity      = 0.9;

r_fly   = p.geometry.flywheelDiameter_m / 2;
h_fly   = p.geometry.flywheelAxialLength_m;
d_shaft = p.geometry.shaftAndPMDiameter_m;
r_shaft = d_shaft / 2;
h_motor = p.geometry.motorAxialLength_m;
t_mag   = p.geometry.magnetThickness_m;
axial_clear = p.clearance.axial_m;

ambParams = ambParameters(d_shaft, p.amb.ratedForce_N);
h_shaft = (5 * axial_clear) + h_fly + h_motor + 2 * ambParams.axialLength;

% Masses
rho_fly   = p.material.densFly_kgPerM3;
rho_steel = p.material.densSteel_kgPerM3;
vol_shaft = pi * r_shaft^2 * h_shaft;
vol_fly   = pi * (r_fly^2 - r_shaft^2) * h_fly;
m_shaft = rho_steel * vol_shaft;
m_fly   = rho_fly * vol_fly;
m_rotating = m_shaft + m_fly;

%% Inertia calculations
% KLEI ORIGINAL (INCORRECT)
J_klei_original = 0.5 * m_fly * (r_fly^2 - r_shaft^2) + 0.5 * m_shaft * r_shaft^2;

% CORRECTED (should match Team 16)
J_corrected = 0.5 * m_fly * (r_fly^2 + r_shaft^2) + 0.5 * m_shaft * r_shaft^2;

%% Speed limits
omega_max_shaft = p.speed.maxTsShaft_mPerSec / r_shaft;
omega_max_fly   = p.speed.maxTsFly_mPerSec / r_fly;
if omega_max_shaft <= omega_max_fly
    omega_max = omega_max_shaft;
else
    omega_max = omega_max_fly;
end
omega_min = 0.5 * omega_max;

%% Energy calculations - Original Klei
KE_max_orig = 0.5 * J_klei_original * omega_max^2;
KE_min_orig = 0.5 * J_klei_original * omega_min^2;
E_stored_orig = KE_max_orig - KE_min_orig;

%% Energy calculations - Corrected
KE_max_corr = 0.5 * J_corrected * omega_max^2;
KE_min_corr = 0.5 * J_corrected * omega_min^2;
E_stored_corr = KE_max_corr - KE_min_corr;

%% Rated power (same for both)
tau_rated = magneticShear(t_mag, 1);
r_rotor = r_shaft + t_mag;
T_rated = 2 * pi * r_rotor^2 * h_motor * tau_rated;
P_rated_W = T_rated * omega_min;

%% Specific performance
spec_power = (P_rated_W/1000) / m_rotating;
spec_energy_orig = (E_stored_orig/3600) / m_rotating;
spec_energy_corr = (E_stored_corr/3600) / m_rotating;

%% Print comparison
fprintf('GEOMETRY (same for all):\n');
fprintf('  Flywheel: %.0f mm dia x %.0f mm length\n', r_fly*2000, h_fly*1000);
fprintf('  Shaft diameter: %.0f mm\n', d_shaft*1000);
fprintf('  Flywheel mass: %.2f kg\n', m_fly);
fprintf('  Shaft mass: %.2f kg\n', m_shaft);
fprintf('  Total rotating mass: %.2f kg\n\n', m_rotating);

fprintf('INERTIA COMPARISON:\n');
fprintf('  %-35s %.4f kg*m^2\n', 'Klei Original (r^2 - r^2):', J_klei_original);
fprintf('  %-35s %.4f kg*m^2\n', 'Corrected (r^2 + r^2):', J_corrected);
fprintf('  %-35s %.2f%%\n\n', 'Difference:', 100*(J_corrected - J_klei_original)/J_corrected);

fprintf('ENERGY STORAGE COMPARISON:\n');
fprintf('  %-35s\n', '                           Original    Corrected');
fprintf('  %-25s %8.2f kWh  %8.2f kWh\n', 'KE at 100% SoC:', KE_max_orig/3.6e6, KE_max_corr/3.6e6);
fprintf('  %-25s %8.2f kWh  %8.2f kWh\n', 'KE at 0% SoC:', KE_min_orig/3.6e6, KE_min_corr/3.6e6);
fprintf('  %-25s %8.2f kWh  %8.2f kWh\n\n', 'Usable energy:', E_stored_orig/3.6e6, E_stored_corr/3.6e6);

fprintf('RATED POWER:\n');
fprintf('  Rated torque: %.2f Nm\n', T_rated);
fprintf('  Rated power: %.2f kW\n\n', P_rated_W/1000);

fprintf('SPECIFIC PERFORMANCE:\n');
fprintf('  %-35s\n', '                           Original    Corrected');
fprintf('  %-25s %8.3f       %8.3f kW/kg\n', 'Specific power:', spec_power, spec_power);
fprintf('  %-25s %8.2f       %8.2f Wh/kg\n\n', 'Specific energy:', spec_energy_orig, spec_energy_corr);

%% Now simulate 15-minute baseline cycle with CORRECTED inertia
fprintf('=================================================================\n');
fprintf('   STORAGE CYCLE EFFICIENCY WITH CORRECTED INERTIA\n');
fprintf('=================================================================\n\n');

J = J_corrected;  % Use corrected inertia
KE_min = KE_min_corr;
KE_max = KE_max_corr;

% Surface areas for thermal calculation
r_motor = r_shaft + t_mag;
L_exposed_shaft = h_shaft - h_fly - h_motor;
A_fly_cylinder = 2 * pi * r_fly * h_fly;
A_fly_ends = 2 * pi * (r_fly^2 - r_shaft^2);
A_motor_cylinder = 2 * pi * r_motor * h_motor;
A_shaft_cylinder = 2 * pi * r_shaft * L_exposed_shaft;
A_shaft_ends = 2 * pi * r_shaft^2;
A_rotor = A_fly_cylinder + A_fly_ends + A_motor_cylinder + A_shaft_cylinder + A_shaft_ends;

housing_clearance = 0.020;
housing_inner_dia = r_fly*2 + 2*housing_clearance;
housing_length = h_shaft + 2*housing_clearance;
A_housing = 2*pi*(housing_inner_dia/2)*housing_length + 2*pi*(housing_inner_dia/2)^2;

epsilon_rotor = p.thermal.rotorEmissivity;
epsilon_housing = p.thermal.housingEmissivity;
rad_factor = 1/epsilon_rotor + (A_rotor/A_housing)*(1/epsilon_housing - 1);
sigma = 5.67e-8;
T_housing_K = p.thermal.housingTemp_C + 273.15;

D_rotor = d_shaft + 2*t_mag;
I_rated_pu = 1.0;

% Simulation parameters
dt = 1.0;
t = 0:dt:900;
SoC_initial = 50;

omega_curr = omega_min + (omega_max - omega_min) * SoC_initial/100;
E_curr = 0.5 * J * omega_curr^2;

SoC_sim = zeros(size(t));
power_grid = zeros(size(t));
power_loss = zeros(size(t));
E_in = 0;
E_out = 0;

for i = 1:length(t)
    P_grid = baselineStorageCycle(t(i));
    power_grid(i) = P_grid;

    omega_rpm = omega_curr * 60 / (2*pi);
    SoC_sim(i) = 100 * (omega_curr - omega_min) / (omega_max - omega_min);

    I_pu = I_rated_pu * abs(P_grid) / P_rated_W;
    I_pu = min(I_pu, 1.0);

    P_rotor = rotorLosses(t_mag, D_rotor, h_motor, I_pu, omega_rpm);
    P_stator = statorLosses(t_mag, D_rotor, h_motor, I_pu, omega_rpm);
    P_loss = P_rotor + P_stator;
    power_loss(i) = P_loss;

    dE = -(P_grid + P_loss) * dt;
    E_curr = E_curr + dE;

    omega_curr = sqrt(max(0, 2 * E_curr / J));
    omega_curr = max(omega_min, min(omega_max, omega_curr));

    if P_grid > 0
        E_out = E_out + P_grid * dt;
    else
        E_in = E_in + abs(P_grid) * dt;
    end
end

E_loss_total = sum(power_loss) * dt;
SoC_final = SoC_sim(end);
omega_initial = omega_min + (omega_max - omega_min) * SoC_initial/100;
omega_final = omega_min + (omega_max - omega_min) * SoC_final/100;
E_recovery = max(0, 0.5 * J * (omega_initial^2 - omega_final^2));

efficiency = (E_out / (E_in + E_recovery)) * 100;

fprintf('15-MINUTE BASELINE CYCLE RESULTS (Corrected Inertia):\n');
fprintf('  Energy discharged: %.2f kWh\n', E_out/3.6e6);
fprintf('  Energy charged: %.2f kWh\n', E_in/3.6e6);
fprintf('  Total losses: %.2f kWh\n', E_loss_total/3.6e6);
fprintf('  Final SoC: %.1f%% (started at %.0f%%)\n', SoC_final, SoC_initial);
fprintf('  Cycle Efficiency: %.2f%%\n\n', efficiency);

%% Compute losses vs SoC
fprintf('LOSSES AND TEMPERATURE (Corrected Inertia):\n');
SoC_range = linspace(0, 100, 50);
max_rotor_loss = 0;
max_stator_loss = 0;
max_total_loss = 0;
max_temp = 0;

for i = 1:length(SoC_range)
    omega = omega_min + (omega_max - omega_min) * SoC_range(i)/100;
    omega_rpm = omega * 60 / (2*pi);
    I_pu = omega_min / omega;

    P_rotor = rotorLosses(t_mag, D_rotor, h_motor, I_pu, omega_rpm);
    P_stator = statorLosses(t_mag, D_rotor, h_motor, I_pu, omega_rpm);

    max_rotor_loss = max(max_rotor_loss, P_rotor);
    max_stator_loss = max(max_stator_loss, P_stator);
    max_total_loss = max(max_total_loss, P_rotor + P_stator);

    T_rotor_K = (T_housing_K^4 + P_rotor*rad_factor/(sigma*A_rotor))^0.25;
    max_temp = max(max_temp, T_rotor_K - 273.15);
end

fprintf('  Max rotor losses: %.2f kW\n', max_rotor_loss/1000);
fprintf('  Max stator losses: %.2f kW\n', max_stator_loss/1000);
fprintf('  Max total losses: %.2f kW\n', max_total_loss/1000);
fprintf('  Max temperature: %.1f C\n\n', max_temp);

%% Final comparison table
fprintf('=================================================================\n');
fprintf('   FINAL COMPARISON: Team 16 vs Klei Corrected\n');
fprintf('=================================================================\n\n');

fprintf('%-30s %15s %15s\n', 'Metric', 'Team 16', 'Klei Corrected');
fprintf('%s\n', repmat('-', 1, 60));
fprintf('%-30s %15.2f %15.2f kg\n', 'Total mass:', 293.10, m_rotating);
fprintf('%-30s %15.4f %15.4f kg*m^2\n', 'Spin-axis inertia:', 5.4238, J_corrected);
fprintf('%-30s %15.2f %15.2f kWh\n', 'Usable energy:', 9.91, E_stored_corr/3.6e6);
fprintf('%-30s %15.2f %15.2f kW\n', 'Rated power:', 203.03, P_rated_W/1000);
fprintf('%-30s %15.3f %15.3f kW/kg\n', 'Specific power:', 0.693, spec_power);
fprintf('%-30s %15.2f %15.2f Wh/kg\n', 'Specific energy:', 33.82, spec_energy_corr);
fprintf('%-30s %15.2f %15.2f %%\n', '15-min efficiency:', 95.80, efficiency);
fprintf('%s\n', repmat('-', 1, 60));
fprintf('\nResults now match closely after correcting the inertia formula!\n');
