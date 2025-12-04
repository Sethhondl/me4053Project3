%% Compare Klei Group Deliverable 1 calculations
% This script extracts key values from the Klei group's approach

addpath('Project3_Functions');

% Use their parameters
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

% KLEI GROUP INERTIA FORMULA - Uses (r_fly^2 - r_shaft^2)
% This is mathematically INCORRECT for a hollow cylinder!
% Correct formula: J = 0.5 * m * (r_outer^2 + r_inner^2)
J_klei = 0.5 * m_fly * (r_fly^2 - r_shaft^2) + 0.5 * m_shaft * r_shaft^2;

% CORRECT INERTIA FORMULA - Uses (r_fly^2 + r_shaft^2)
J_correct = 0.5 * m_fly * (r_fly^2 + r_shaft^2) + 0.5 * m_shaft * r_shaft^2;

% Speed limits
omega_max_shaft = p.speed.maxTsShaft_mPerSec / r_shaft;
omega_max_fly   = p.speed.maxTsFly_mPerSec / r_fly;
if omega_max_shaft <= omega_max_fly
    omega_max = omega_max_shaft;
else
    omega_max = omega_max_fly;
end
omega_min = 0.5 * omega_max;

% Energy calculations
KE_max_klei = 0.5 * J_klei * omega_max^2;
KE_min_klei = 0.5 * J_klei * omega_min^2;
E_stored_klei = KE_max_klei - KE_min_klei;

KE_max_correct = 0.5 * J_correct * omega_max^2;
KE_min_correct = 0.5 * J_correct * omega_min^2;
E_stored_correct = KE_max_correct - KE_min_correct;

% Rated power
tau_rated = magneticShear(t_mag, 1);
r_rotor = r_shaft + t_mag;
T_rated = 2 * pi * r_rotor^2 * h_motor * tau_rated;
P_rated_W = T_rated * omega_min;

fprintf('=== KLEI GROUP DELIVERABLE 1 KEY VALUES ===\n\n');

fprintf('GEOMETRY:\n');
fprintf('  Flywheel mass: %.2f kg\n', m_fly);
fprintf('  Shaft mass: %.2f kg\n', m_shaft);
fprintf('  Total rotating mass: %.2f kg\n', m_rotating);
fprintf('  Shaft length: %.3f m\n\n', h_shaft);

fprintf('SPEED LIMITS:\n');
fprintf('  omega_max: %.2f rad/s (%.0f rpm)\n', omega_max, omega_max*60/(2*pi));
fprintf('  omega_min: %.2f rad/s (%.0f rpm)\n\n', omega_min, omega_min*60/(2*pi));

fprintf('INERTIA COMPARISON:\n');
fprintf('  KLEI formula J = 0.5*m*(r_o^2 - r_i^2): %.4f kg*m^2\n', J_klei);
fprintf('  CORRECT formula J = 0.5*m*(r_o^2 + r_i^2): %.4f kg*m^2\n', J_correct);
fprintf('  Difference: %.2f%%\n\n', 100*(J_correct - J_klei)/J_correct);

fprintf('ENERGY STORAGE (using Klei formula):\n');
fprintf('  KE_max: %.2f kWh\n', KE_max_klei/3.6e6);
fprintf('  KE_min: %.2f kWh\n', KE_min_klei/3.6e6);
fprintf('  Usable Energy: %.2f kWh\n\n', E_stored_klei/3.6e6);

fprintf('ENERGY STORAGE (using correct formula):\n');
fprintf('  KE_max: %.2f kWh\n', KE_max_correct/3.6e6);
fprintf('  KE_min: %.2f kWh\n', KE_min_correct/3.6e6);
fprintf('  Usable Energy: %.2f kWh\n\n', E_stored_correct/3.6e6);

fprintf('RATED POWER:\n');
fprintf('  Torque: %.2f Nm\n', T_rated);
fprintf('  Rated power: %.2f kW\n\n', P_rated_W/1000);

fprintf('SPECIFIC PERFORMANCE (Klei values):\n');
fprintf('  Specific power: %.3f kW/kg\n', (P_rated_W/1000)/m_rotating);
fprintf('  Specific energy: %.2f Wh/kg\n\n', (E_stored_klei/3600)/m_rotating);

fprintf('SPECIFIC PERFORMANCE (Correct values):\n');
fprintf('  Specific power: %.3f kW/kg\n', (P_rated_W/1000)/m_rotating);
fprintf('  Specific energy: %.2f Wh/kg\n', (E_stored_correct/3600)/m_rotating);
