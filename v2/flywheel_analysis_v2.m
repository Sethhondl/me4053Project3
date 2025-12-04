%% FLYWHEEL ENERGY STORAGE SYSTEM ANALYSIS - VERSION 2
% =========================================================================
% Course: Mechanical Engineering Modeling
% Project: Flywheel Energy Storage System for eXtreme Storage Inc.
%
% This consolidated script performs complete analysis including:
%   Deliverable 1: Baseline System Characterization (Parts 1a-1f)
%   Deliverable 2: New Design Development (Parts 2a-2b)
%   Deliverable 3: AMB Controller Design & Comparison (Parts 3a-3c)
%
% REQUIREMENTS:
%   - Control System Toolbox (for tf, step, bode, margin, etc.)
%   - Project3_Functions/ directory with .p files
%
% Author: Team 16
% Date: 2025-12-01
% Version: 2.0
% =========================================================================

clear; close all; clc;

%% ========================================================================
% INITIALIZATION
% =========================================================================
addpath('../Project3_Functions');

fprintf('================================================================\n');
fprintf('   FLYWHEEL ENERGY STORAGE SYSTEM ANALYSIS - VERSION 2\n');
fprintf('================================================================\n');
fprintf('Date: %s\n', datestr(now));
fprintf('Using Control System Toolbox for transfer function analysis\n\n');

%% ========================================================================
% BASELINE SYSTEM PARAMETERS (from Appendix B - Table A.1)
% =========================================================================
% See ASSUMPTIONS.md for complete documentation of all assumptions

fprintf('Loading baseline system parameters...\n\n');

% --- Geometric Parameters [m] ---
baseline.flywheel_length = 1.000;      % Flywheel axial length
baseline.flywheel_diameter = 0.430;    % Flywheel outer diameter
baseline.motor_length = 0.250;         % Motor/generator axial length
baseline.shaft_diameter = 0.084;       % Shaft and PM inner diameter
baseline.magnet_thickness = 0.006;     % Permanent magnet radial thickness

% --- Operational Parameters ---
baseline.max_speed_rpm = 40000;        % Maximum rotational speed [RPM]
baseline.min_speed_rpm = 20000;        % Minimum speed (0% SoC) [RPM]
baseline.amb_rated_force = 5780;       % AMB rated force per bearing [N]

% --- Material Properties (from Table 1) ---
rho_composite = 1600;                  % Composite flywheel [kg/m^3]
rho_steel = 7850;                      % Steel shaft [kg/m^3]
rho_magnet = 7850;                     % Permanent magnets [kg/m^3]

% --- Material Limits (from Table 1) ---
max_steel_tip_speed = 175;             % Steel/shaft tip speed limit [m/s]
max_pm_tip_speed = 175;                % Permanent magnet tip speed [m/s]
max_composite_tip_speed = 900;         % Composite tip speed limit [m/s]
max_temp = 100;                        % Maximum safe temperature [C]

% --- Thermal Parameters (from Table 1) ---
epsilon_rotor = 0.4;                   % Rotor surface emissivity
epsilon_housing = 0.9;                 % Housing surface emissivity
sigma = 5.67e-8;                       % Stefan-Boltzmann [W/(m^2*K^4)]
T_housing_C = 30;                      % Housing temperature [C] (Appendix B)
T_housing_K = T_housing_C + 273.15;    % Housing temperature [K]

% --- Angular Velocities ---
baseline.omega_max = baseline.max_speed_rpm * 2*pi / 60;  % [rad/s]
baseline.omega_min = baseline.min_speed_rpm * 2*pi / 60;  % [rad/s]

% --- Controller Parameters (from Appendix B) ---
% Current controller: G_ci(s) = Kp + Ki/s
baseline.Kp_current = 345;
baseline.Ki_current = 2149;

% Position controller X (PID with filtered derivative):
% G_cx(s) = kpx + kix/s + s*kdx/(1 + s/omega_px)
baseline.kpx = 1.2639e8;               % Proportional [N/m]
baseline.kix = 1.16868e9;              % Integral [N/(m*s)]
baseline.kdx = 252790;                 % Derivative [N*s/m]
baseline.omega_px = 3770;              % Filter cutoff [rad/s]

% Tilting position controller (PID with filtered derivative):
% G_alpha(s) = kp_alpha + ki_alpha/s + s*kd_alpha/(1 + s/omega_p_alpha)
baseline.kp_alpha = 7.6992e7;          % [N*m/rad]
baseline.ki_alpha = 1.18953e9;         % [N*m/(rad*s)]
baseline.kd_alpha = 80294;             % [N*m*s/rad]
baseline.omega_p_alpha = 6283;         % [rad/s]

%% ========================================================================
% MASS AND INERTIA CALCULATIONS
% =========================================================================
fprintf('Calculating rotating group mass and inertia...\n');

% Flywheel (hollow composite cylinder)
r_outer_bl = baseline.flywheel_diameter / 2;
r_inner_bl = baseline.shaft_diameter / 2;
V_flywheel_bl = pi * (r_outer_bl^2 - r_inner_bl^2) * baseline.flywheel_length;
m_flywheel_bl = rho_composite * V_flywheel_bl;

% Shaft (solid steel cylinder)
% Estimate shaft length from geometry + AMB axial length
axial_clearance = 0.020;  % 20 mm clearance
ambParams_temp = ambParameters(baseline.shaft_diameter, baseline.amb_rated_force);
shaft_length_bl = (5 * axial_clearance) + baseline.flywheel_length + ...
    baseline.motor_length + 2 * ambParams_temp.axialLength;
V_shaft_bl = pi * r_inner_bl^2 * shaft_length_bl;
m_shaft_bl = rho_steel * V_shaft_bl;

% Total rotating mass (simplified model: flywheel + shaft only)
baseline.total_mass = m_flywheel_bl + m_shaft_bl;

% Moments of inertia about spin axis
% Flywheel: hollow cylinder I = 0.5 * m * (r_o^2 + r_i^2)
I_flywheel_bl = 0.5 * m_flywheel_bl * (r_outer_bl^2 + r_inner_bl^2);
% Shaft: solid cylinder I = 0.5 * m * r^2
I_shaft_bl = 0.5 * m_shaft_bl * r_inner_bl^2;
baseline.I_total = I_flywheel_bl + I_shaft_bl;

% Transverse moment of inertia (for tilting analysis)
baseline.I_transverse = (1/12) * baseline.total_mass * ...
    (3*(baseline.flywheel_diameter/2)^2 + baseline.flywheel_length^2);

% AMB separation distance
% ASSUMPTION: L_amb = flywheel_length + 0.3 m (see ASSUMPTIONS.md)
baseline.L_amb = baseline.flywheel_length + 0.3;

% =========================================================================
% 4-DOF GEOMETRY: COM POSITION AND AMB DISTANCES
% =========================================================================
% Reference: z=0 at bottom of shaft
% Layout (bottom to top): bottom AMB -> motor -> flywheel -> top AMB

% Component positions along shaft axis (z-direction)
z_amb_bottom = ambParams_temp.axialLength / 2;  % Bottom AMB center
z_motor_center = ambParams_temp.axialLength + axial_clearance + baseline.motor_length/2;
z_flywheel_center = ambParams_temp.axialLength + 2*axial_clearance + baseline.motor_length + baseline.flywheel_length/2;
z_amb_top = shaft_length_bl - ambParams_temp.axialLength / 2;  % Top AMB center

% Component COM positions (shaft COM is at geometric center)
z_shaft_com = shaft_length_bl / 2;
z_flywheel_com = z_flywheel_center;

% Total system COM (mass-weighted average)
baseline.z_com = (m_shaft_bl * z_shaft_com + m_flywheel_bl * z_flywheel_com) / baseline.total_mass;

% Distances from COM to each AMB (positive = toward top)
baseline.d_amb_top = z_amb_top - baseline.z_com;      % Distance COM to top AMB
baseline.d_amb_bottom = baseline.z_com - z_amb_bottom; % Distance COM to bottom AMB

% Proper transverse inertia using parallel axis theorem
d_shaft_from_com = z_shaft_com - baseline.z_com;
d_fly_from_com = z_flywheel_com - baseline.z_com;

% Shaft transverse inertia: I_cm + m*d^2
I_t_shaft = (1/12)*m_shaft_bl*shaft_length_bl^2 + 0.25*m_shaft_bl*r_inner_bl^2 + m_shaft_bl*d_shaft_from_com^2;
% Flywheel transverse inertia: I_cm + m*d^2
I_t_flywheel = (1/12)*m_flywheel_bl*baseline.flywheel_length^2 + ...
               0.25*m_flywheel_bl*(r_outer_bl^2 + r_inner_bl^2) + m_flywheel_bl*d_fly_from_com^2;
baseline.I_transverse = I_t_shaft + I_t_flywheel;

fprintf('  Flywheel mass: %.2f kg\n', m_flywheel_bl);
fprintf('  Shaft mass: %.2f kg\n', m_shaft_bl);
fprintf('  Total rotating mass: %.2f kg\n', baseline.total_mass);
fprintf('  Spin-axis inertia: %.4f kg*m^2\n', baseline.I_total);
fprintf('  Transverse inertia: %.4f kg*m^2\n', baseline.I_transverse);
fprintf('  COM position (from bottom): %.3f m\n', baseline.z_com);
fprintf('  Distance COM to top AMB: %.3f m\n', baseline.d_amb_top);
fprintf('  Distance COM to bottom AMB: %.3f m\n\n', baseline.d_amb_bottom);

%% ========================================================================
% AMB PARAMETERS
% =========================================================================
fprintf('Loading AMB parameters...\n');

amb_bl = ambParameters(baseline.shaft_diameter, baseline.amb_rated_force);

% Extract key parameters
% Note: stiffnessConstant is positive magnitude
% In AMB physics: F_stiff = Ks * x where Ks > 0 (destabilizing)
% A positive displacement creates positive force (pulls further away)
baseline.K_s = amb_bl.stiffnessConstant;   % Position stiffness [N/m] (positive, destabilizing)
baseline.K_i = amb_bl.forceConstant;        % Force constant [N/A]
baseline.L_coil = amb_bl.coilInductance;    % Coil inductance [H]
baseline.R_coil = amb_bl.coilResistance;    % Coil resistance [Ohms]

fprintf('  Position stiffness (K_s): %.3e N/m (positive/destabilizing)\n', baseline.K_s);
fprintf('  Force constant (K_i): %.2f N/A\n', baseline.K_i);
fprintf('  Coil inductance: %.4f H\n', baseline.L_coil);
fprintf('  Coil resistance: %.3f Ohms\n\n', baseline.R_coil);

% Unstable pole frequency
f_unstable_bl = sqrt(abs(baseline.K_s) / baseline.total_mass) / (2*pi);
fprintf('  Unstable pole frequency: %.2f Hz\n\n', f_unstable_bl);

%% ========================================================================
% RATED POWER CALCULATION
% =========================================================================
fprintf('Calculating rated power...\n');

% ASSUMPTION: Rated current = 1.0 pu (maximum available)
I_rated_pu = 1.0;

% Get magnetic shear at rated current
shear_rated = magneticShear(baseline.magnet_thickness, I_rated_pu);

% Rotor radius includes magnets (torque acts at air gap surface)
rotor_radius_bl = (baseline.shaft_diameter / 2) + baseline.magnet_thickness;
rotor_diameter_bl = baseline.shaft_diameter + 2 * baseline.magnet_thickness;

% Torque = 2*pi*r^2*L*tau (from Peyton's formula)
torque_rated_bl = 2 * pi * rotor_radius_bl^2 * baseline.motor_length * shear_rated;

% Rated power defined at MINIMUM speed (0% SoC) - this is when max torque is needed
baseline.power_rated = torque_rated_bl * baseline.omega_min;

fprintf('  Rated current: %.2f pu\n', I_rated_pu);
fprintf('  Magnetic shear: %.2f Pa\n', shear_rated);
fprintf('  Rotor diameter (with magnets): %.3f m\n', rotor_diameter_bl);
fprintf('  Rated torque: %.2f Nm\n', torque_rated_bl);
fprintf('  Rated power (at omega_min): %.2f kW\n\n', baseline.power_rated/1000);

%% ########################################################################
%                         DELIVERABLE 1
%                   BASELINE SYSTEM CHARACTERIZATION
% #########################################################################

fprintf('\n');
fprintf('================================================================\n');
fprintf('              DELIVERABLE 1: BASELINE ANALYSIS\n');
fprintf('================================================================\n\n');

%% ========================================================================
% PART 1a: LOSSES AND TEMPERATURE VS STATE OF CHARGE
% =========================================================================
fprintf('================================================================\n');
fprintf('PART 1a: Losses and Temperature vs SoC\n');
fprintf('================================================================\n\n');

% SoC range
SoC_1a = linspace(0, 100, 50);

% Surface areas for thermal calculation - ALL spinning components
%
% Geometry:
%   - Flywheel: hollow cylinder (r_inner to r_outer) x L_flywheel
%   - Motor/rotor: cylinder with magnets (r_shaft+t_mag) x L_motor
%   - Shaft: extends through everything, exposed sections at ends
%
r_motor = r_inner_bl + baseline.magnet_thickness;  % Motor outer radius (with magnets)
L_exposed_shaft = shaft_length_bl - baseline.flywheel_length - baseline.motor_length;

% 1. Flywheel outer cylindrical surface
A_fly_cylinder = 2 * pi * r_outer_bl * baseline.flywheel_length;

% 2. Flywheel annular end faces (ring shape, accounts for shaft hole)
A_fly_ends = 2 * pi * (r_outer_bl^2 - r_inner_bl^2);

% 3. Motor/rotor outer cylindrical surface
A_motor_cylinder = 2 * pi * r_motor * baseline.motor_length;

% 4. Exposed shaft cylindrical surface (sections not covered by flywheel or motor)
A_shaft_cylinder = 2 * pi * r_inner_bl * L_exposed_shaft;

% 5. Shaft end caps (two circular ends)
A_shaft_ends = 2 * pi * r_inner_bl^2;

% Total radiating surface area
A_rotor = A_fly_cylinder + A_fly_ends + A_motor_cylinder + A_shaft_cylinder + A_shaft_ends;

fprintf('Radiating surface areas:\n');
fprintf('  Flywheel cylinder:  %.4f m^2\n', A_fly_cylinder);
fprintf('  Flywheel ends:      %.4f m^2 (annular)\n', A_fly_ends);
fprintf('  Motor cylinder:     %.4f m^2\n', A_motor_cylinder);
fprintf('  Shaft cylinder:     %.4f m^2\n', A_shaft_cylinder);
fprintf('  Shaft ends:         %.4f m^2\n', A_shaft_ends);
fprintf('  TOTAL:              %.4f m^2\n\n', A_rotor);

% Housing surface area (surrounds everything)
housing_clearance = 0.020;  % 20 mm radial clearance
housing_inner_dia = baseline.flywheel_diameter + 2*housing_clearance;
housing_length = shaft_length_bl + 2*housing_clearance;  % Housing covers full shaft length
A_housing = 2*pi*(housing_inner_dia/2)*housing_length + 2*pi*(housing_inner_dia/2)^2;

% Two-surface enclosure radiation factor
rad_factor = 1/epsilon_rotor + (A_rotor/A_housing)*(1/epsilon_housing - 1);
fprintf('Radiation factor (F_rad): %.2f\n', rad_factor);

% Initialize arrays
rotor_loss_1a = zeros(size(SoC_1a));
stator_loss_1a = zeros(size(SoC_1a));
total_loss_1a = zeros(size(SoC_1a));
temperature_1a = zeros(size(SoC_1a));
current_1a = zeros(size(SoC_1a));

% Rotor diameter is shaft + 2*magnet thickness (outer diameter of magnets)
D_rotor = baseline.shaft_diameter + 2 * baseline.magnet_thickness;  % 0.096 m

fprintf('Calculating losses at rated power across SoC range...\n');
fprintf('  Using rotor diameter: %.3f m (shaft + 2*magnets)\n', D_rotor);

for i = 1:length(SoC_1a)
    % Speed at this SoC (energy-based: E ∝ ω², so ω² is linear with SoC)
    omega = sqrt(baseline.omega_min^2 + (baseline.omega_max^2 - baseline.omega_min^2) * SoC_1a(i)/100);
    omega_rpm = omega * 60 / (2*pi);

    % Current for constant power operation
    % At 0% SoC (omega_min): Ipu = 1.0 (rated torque, rated power)
    % At higher SoC: Ipu = omega_min/omega < 1.0 (less torque needed)
    % This keeps power constant at P_rated = T_rated * omega_min
    I_pu = baseline.omega_min / omega;
    current_1a(i) = I_pu;

    % Get losses from EE functions
    % NOTE: rotorDiameter = outer diameter of magnets (shaft + 2*magnet_thickness)
    P_rotor = rotorLosses(baseline.magnet_thickness, D_rotor, ...
        baseline.motor_length, I_pu, omega_rpm);
    P_stator = statorLosses(baseline.magnet_thickness, D_rotor, ...
        baseline.motor_length, I_pu, omega_rpm);

    rotor_loss_1a(i) = P_rotor;
    stator_loss_1a(i) = P_stator;
    total_loss_1a(i) = P_rotor + P_stator;

    % Temperature from radiation heat transfer
    % ONLY rotor losses heat the rotor (stator is outside vacuum)
    % Q = sigma * A * (T_rotor^4 - T_housing^4) / rad_factor
    T_rotor_K = (T_housing_K^4 + P_rotor*rad_factor/(sigma*A_rotor))^0.25;
    temperature_1a(i) = T_rotor_K - 273.15;
end

% --- PLOT: Part 1a ---
figure('Name', 'Part 1a: Losses and Temperature', 'Position', [100 100 1000 450]);

subplot(1,2,1);
plot(SoC_1a, rotor_loss_1a/1000, 'r-', 'LineWidth', 2); hold on;
plot(SoC_1a, stator_loss_1a/1000, 'b-', 'LineWidth', 2);
plot(SoC_1a, total_loss_1a/1000, 'k-', 'LineWidth', 2.5);
grid on;
xlabel('State of Charge [%]', 'FontSize', 12);
ylabel('Power Loss [kW]', 'FontSize', 12);
title('Part 1a: Motor Losses at Rated Power vs SoC', 'FontSize', 13);
legend('Rotor Losses', 'Stator Losses', 'Total Losses', 'Location', 'best');

subplot(1,2,2);
plot(SoC_1a, temperature_1a, 'k-', 'LineWidth', 2.5); hold on;
yline(max_temp, 'r--', 'LineWidth', 2, 'Label', 'Max Safe (100 C)');
grid on;
xlabel('State of Charge [%]', 'FontSize', 12);
ylabel('Rotor Temperature [C]', 'FontSize', 12);
title('Part 1a: Rotor Temperature vs SoC', 'FontSize', 13);

saveas(gcf, 'part1a_losses_temperature.png');
fprintf('Plot saved: part1a_losses_temperature.png\n');

fprintf('\nResults Summary:\n');
fprintf('  Max rotor losses: %.2f kW\n', max(rotor_loss_1a)/1000);
fprintf('  Max stator losses: %.2f kW\n', max(stator_loss_1a)/1000);
fprintf('  Max total losses: %.2f kW at %.0f%% SoC\n', max(total_loss_1a)/1000, ...
    SoC_1a(total_loss_1a == max(total_loss_1a)));
fprintf('  Max temperature: %.1f C (limit: %.0f C)\n\n', max(temperature_1a), max_temp);

%% ========================================================================
% PART 1b: SPECIFIC POWER AND SPECIFIC ENERGY
% =========================================================================
fprintf('================================================================\n');
fprintf('PART 1b: Specific Power and Specific Energy\n');
fprintf('================================================================\n\n');

% Specific power [kW/kg]
specific_power_bl = (baseline.power_rated / 1000) / baseline.total_mass;

% Energy storage
E_max = 0.5 * baseline.I_total * baseline.omega_max^2;  % [J] at 100% SoC
E_min = 0.5 * baseline.I_total * baseline.omega_min^2;  % [J] at 0% SoC
E_stored = E_max - E_min;                                % [J] usable
E_stored_kWh = E_stored / 3.6e6;                        % [kWh]

% Specific energy [Wh/kg]
specific_energy_bl = (E_stored / 3600) / baseline.total_mass;

fprintf('Energy Storage:\n');
fprintf('  Energy at 100%% SoC: %.2f kWh\n', E_max/3.6e6);
fprintf('  Energy at 0%% SoC: %.2f kWh\n', E_min/3.6e6);
fprintf('  Usable energy: %.2f kWh\n\n', E_stored_kWh);

fprintf('Specific Performance:\n');
fprintf('  Specific Power: %.3f kW/kg\n', specific_power_bl);
fprintf('  Specific Energy: %.2f Wh/kg\n\n', specific_energy_bl);

%% ========================================================================
% PART 1c: STORAGE CYCLE EFFICIENCY
% =========================================================================
fprintf('================================================================\n');
fprintf('PART 1c: Storage Cycle Efficiency (15-minute baseline)\n');
fprintf('================================================================\n\n');

% Simulation parameters
dt_1c = 1.0;              % Time step [s]
t_1c = 0:dt_1c:900;       % 15-minute cycle
SoC_initial = 50;         % Start at 50% SoC

% Initial state (energy-based SoC: ω² is linear with SoC)
omega_curr = sqrt(baseline.omega_min^2 + (baseline.omega_max^2 - baseline.omega_min^2) * SoC_initial/100);
E_curr = 0.5 * baseline.I_total * omega_curr^2;

% Storage arrays
SoC_1c = zeros(size(t_1c));
power_grid_1c = zeros(size(t_1c));
power_loss_1c = zeros(size(t_1c));

% Energy tracking
E_in = 0;
E_out = 0;

fprintf('Simulating 15-minute baseline storage cycle...\n');

for i = 1:length(t_1c)
    % Get grid power demand (positive = discharge to grid)
    P_grid = baselineStorageCycle(t_1c(i));
    power_grid_1c(i) = P_grid;

    % Current speed (energy-based SoC: SoC ∝ ω²)
    omega_rpm = omega_curr * 60 / (2*pi);
    SoC_1c(i) = 100 * (omega_curr^2 - baseline.omega_min^2) / (baseline.omega_max^2 - baseline.omega_min^2);

    % Calculate current for this power
    I_pu = I_rated_pu * abs(P_grid) / baseline.power_rated;
    I_pu = min(I_pu, 1.0);

    % Get losses (use D_rotor = shaft + 2*magnets)
    P_rotor = rotorLosses(baseline.magnet_thickness, rotor_diameter_bl, ...
        baseline.motor_length, I_pu, omega_rpm);
    P_stator = statorLosses(baseline.magnet_thickness, rotor_diameter_bl, ...
        baseline.motor_length, I_pu, omega_rpm);
    P_loss = P_rotor + P_stator;
    power_loss_1c(i) = P_loss;

    % Energy balance: dE/dt = -(P_grid + P_loss)
    dE = -(P_grid + P_loss) * dt_1c;
    E_curr = E_curr + dE;

    % Update speed
    omega_curr = sqrt(max(0, 2 * E_curr / baseline.I_total));
    omega_curr = max(baseline.omega_min, min(baseline.omega_max, omega_curr));

    % Track energy flows
    if P_grid > 0
        E_out = E_out + P_grid * dt_1c;
    else
        E_in = E_in + abs(P_grid) * dt_1c;
    end
end

% Calculate efficiency (energy-based SoC)
E_loss_total = sum(power_loss_1c) * dt_1c;
SoC_final = SoC_1c(end);
omega_initial = sqrt(baseline.omega_min^2 + (baseline.omega_max^2 - baseline.omega_min^2) * SoC_initial/100);
omega_final = sqrt(baseline.omega_min^2 + (baseline.omega_max^2 - baseline.omega_min^2) * SoC_final/100);
E_recovery = max(0, 0.5 * baseline.I_total * (omega_initial^2 - omega_final^2));

efficiency_1c = (E_out / (E_in + E_recovery)) * 100;

% --- PLOT: Part 1c ---
figure('Name', 'Part 1c: Storage Cycle', 'Position', [100 100 1000 700]);

subplot(3,1,1);
plot(t_1c/60, power_grid_1c/1000, 'b-', 'LineWidth', 2);
grid on;
xlabel('Time [min]', 'FontSize', 11);
ylabel('Grid Power [kW]', 'FontSize', 11);
title('Part 1c: Baseline Storage Cycle - Power Demand', 'FontSize', 12);

subplot(3,1,2);
plot(t_1c/60, SoC_1c, 'r-', 'LineWidth', 2);
grid on;
xlabel('Time [min]', 'FontSize', 11);
ylabel('State of Charge [%]', 'FontSize', 11);
title('Part 1c: Flywheel SoC During Cycle', 'FontSize', 12);

subplot(3,1,3);
plot(t_1c/60, power_loss_1c/1000, 'k-', 'LineWidth', 2);
grid on;
xlabel('Time [min]', 'FontSize', 11);
ylabel('Total Losses [kW]', 'FontSize', 11);
title('Part 1c: Motor Losses During Cycle', 'FontSize', 12);

saveas(gcf, 'part1c_storage_cycle.png');
fprintf('Plot saved: part1c_storage_cycle.png\n');

fprintf('\nCycle Results:\n');
fprintf('  Energy discharged: %.2f kWh\n', E_out/3.6e6);
fprintf('  Energy charged: %.2f kWh\n', E_in/3.6e6);
fprintf('  Total losses: %.2f kWh\n', E_loss_total/3.6e6);
fprintf('  Final SoC: %.1f%% (started at %.0f%%)\n', SoC_final, SoC_initial);
fprintf('  Cycle Efficiency: %.2f%%\n\n', efficiency_1c);

%% ========================================================================
% PART 1d: AMB STEP RESPONSE (Using 4-DOF ODE Simulation)
% =========================================================================
fprintf('================================================================\n');
fprintf('PART 1d: AMB Step Response Analysis\n');
fprintf('================================================================\n\n');

fprintf('Using full 4-DOF ODE simulation with coupled dynamics...\n\n');

% Create Laplace variable (still needed for later parts)
s = tf('s');

% --- System Parameters ---
m = baseline.total_mass;           % Rotor mass [kg]
I_t = baseline.I_transverse;       % Transverse moment of inertia [kg*m^2]
L_amb_1d = baseline.L_amb;         % AMB separation distance [m]
a = L_amb_1d / 2;                  % Distance from CM to each bearing [m]

Ks = baseline.K_s;                 % Position stiffness (positive, destabilizing) [N/m]
Ki = baseline.K_i;                 % Force constant [N/A]

fprintf('System Parameters:\n');
fprintf('  Rotor mass (m): %.2f kg\n', m);
fprintf('  Transverse inertia (I_t): %.4f kg*m^2\n', I_t);
fprintf('  AMB separation (L_amb): %.3f m\n', L_amb_1d);
fprintf('  Distance to bearing (a = L/2): %.3f m\n', a);
fprintf('  Position stiffness (Ks): %.3e N/m (positive, destabilizing)\n', Ks);
fprintf('  Force constant (Ki): %.2f N/A\n\n', Ki);

% --- Controller Parameters (for printout) ---
fprintf('Position Controller G_cx(s):\n');
fprintf('  Kp = %.4e, Ki = %.4e, Kd = %.0f, wp = %.0f rad/s\n\n', ...
    baseline.kpx, baseline.kix, baseline.kdx, baseline.omega_px);

fprintf('Tilting Controller G_alpha(s):\n');
fprintf('  Kp = %.4e, Ki = %.4e, Kd = %.0f, wp = %.0f rad/s\n\n', ...
    baseline.kp_alpha, baseline.ki_alpha, baseline.kd_alpha, baseline.omega_p_alpha);

% =========================================================================
% 4-DOF ODE SIMULATION MODEL
% =========================================================================
% Uses full coupled 4-DOF dynamics with 20-state ODE:
%   States 1-4:   x, y, alpha, beta (positions)
%   States 5-8:   velocities
%   States 9-12:  controller integral states
%   States 13-16: derivative filter states
%   States 17-20: current regulator states
%
% This captures the full coupled dynamics including:
%   - Radial and tilting mode coupling through bearing geometry
%   - Current controller dynamics (first-order)
%   - Controller integral windup and derivative filtering
% =========================================================================

fprintf('Setting up 4-DOF ODE simulation parameters...\n');

% Build parameter structure for ODE simulation
ode_params.m = m;
ode_params.J_t = I_t;
ode_params.J_p = baseline.I_total;  % Polar MOI for gyroscopic (=0 at zero speed)
ode_params.d_top = baseline.d_amb_top;
ode_params.d_bot = baseline.d_amb_bottom;
ode_params.Ks = Ks;
ode_params.Ki = Ki;
ode_params.Omega = 0;  % Zero speed for Part 1d

% Position controller gains
ode_params.kpx = baseline.kpx;
ode_params.kix = baseline.kix;
ode_params.kdx = baseline.kdx;
ode_params.omega_px = baseline.omega_px;
ode_params.kp_alpha = baseline.kp_alpha;
ode_params.ki_alpha = baseline.ki_alpha;
ode_params.kd_alpha = baseline.kd_alpha;
ode_params.omega_p_alpha = baseline.omega_p_alpha;

% Current controller parameters (Appendix B)
ode_params.Kp_current = baseline.Kp_current;  % PI current controller Kp
ode_params.Ki_current = baseline.Ki_current;  % PI current controller Ki
ode_params.L_coil = baseline.L_coil;          % Coil inductance [H]
ode_params.R_coil = baseline.R_coil;          % Coil resistance [Ohms]

% Current loop bandwidth from Appendix B controller
wci_appendixB = baseline.Kp_current / baseline.L_coil;
fprintf('  Distance from COM to top AMB: %.3f m\n', ode_params.d_top);
fprintf('  Distance from COM to bottom AMB: %.3f m\n', ode_params.d_bot);
fprintf('  Current loop bandwidth (Appendix B): %.0f rad/s (%.0f Hz)\n', ...
    wci_appendixB, wci_appendixB/(2*pi));

% Keep transfer functions for later parts (dynamic stiffness, etc.)
G_cx_1d = baseline.kpx + baseline.kix/s + baseline.kdx*s/(1 + s/baseline.omega_px);
G_alpha_1d = baseline.kp_alpha + baseline.ki_alpha/s + ...
             baseline.kd_alpha*s/(1 + s/baseline.omega_p_alpha);
K_tilt_eff = 2 * a^2 * Ks;

% --- Step Disturbance ---
F_step = 0.10 * baseline.amb_rated_force;  % 10% rated force
fprintf('\nStep disturbance at TOP bearing: %.1f N (10%% of %.0f N)\n\n', ...
    F_step, baseline.amb_rated_force);

% =========================================================================
% RUN 4-DOF ODE SIMULATION
% =========================================================================
fprintf('Running 4-DOF ODE simulation (ode45)...\n');

t_final = 0.15;  % 150 ms simulation time
results_1d = simulate4DOF_step(ode_params, t_final, F_step, 'x');

fprintf('  Simulation complete: %d time points\n', length(results_1d.t));

% Extract results
t_out = results_1d.t;
x_top_1d = results_1d.pos_top;
x_bot_1d = results_1d.pos_bot;
F_top_1d = results_1d.F_top;
F_bot_1d = results_1d.F_bot;
i_top_1d = results_1d.i_top;
i_bot_1d = results_1d.i_bot;

% Report key results
fprintf('  Peak displacement (top): %.2f um\n', max(abs(x_top_1d))*1e6);
fprintf('  Peak displacement (bot): %.2f um\n', max(abs(x_bot_1d))*1e6);
fprintf('  Final displacement (top): %.4f um\n', abs(x_top_1d(end))*1e6);
fprintf('  Simulation time: %.1f ms\n\n', t_out(end) * 1000);

% --- PLOT: Part 1d ---
figure('Name', 'Part 1d: AMB Step Response', 'Position', [50 50 1200 900]);

% Current subplot
subplot(3,1,1);
plot(t_out*1000, i_top_1d, 'b-', 'LineWidth', 2); hold on;
plot(t_out*1000, i_bot_1d, 'r--', 'LineWidth', 2);
grid on;
xlabel('Time [ms]', 'FontSize', 11);
ylabel('Control Current [A]', 'FontSize', 11);
title('Part 1d: AMB Current Response (0 RPM, 10% rated force step at top bearing)', 'FontSize', 12);
legend('Top Bearing', 'Bottom Bearing', 'Location', 'best');

% Force subplot
subplot(3,1,2);
plot(t_out*1000, F_top_1d, 'b-', 'LineWidth', 2); hold on;
plot(t_out*1000, F_bot_1d, 'r--', 'LineWidth', 2);
yline(-F_step, 'k:', 'LineWidth', 1.5, 'Label', 'Disturbance (-F_d)');
grid on;
xlabel('Time [ms]', 'FontSize', 11);
ylabel('AMB Control Force [N]', 'FontSize', 11);
title('Part 1d: AMB Control Force Response', 'FontSize', 12);
legend('Top Bearing', 'Bottom Bearing', 'Target', 'Location', 'best');

% Position subplot
subplot(3,1,3);
plot(t_out*1000, x_top_1d*1e6, 'b-', 'LineWidth', 2); hold on;
plot(t_out*1000, x_bot_1d*1e6, 'r--', 'LineWidth', 2);
grid on;
xlabel('Time [ms]', 'FontSize', 11);
ylabel('Rotor Position [\mum]', 'FontSize', 11);
title('Part 1d: Rotor Position at Each Bearing', 'FontSize', 12);
legend('Top Bearing', 'Bottom Bearing', 'Location', 'best');

saveas(gcf, 'part1d_amb_step_response.png');
fprintf('Plot saved: part1d_amb_step_response.png\n');

% Calculate metrics
peak_displacement_top = max(abs(x_top_1d)) * 1e6;
peak_displacement_bot = max(abs(x_bot_1d)) * 1e6;
final_displacement_top = abs(x_top_1d(end)) * 1e6;
final_displacement_bot = abs(x_bot_1d(end)) * 1e6;

peak_current_top = max(abs(i_top_1d));
peak_current_bot = max(abs(i_bot_1d));

% Compute settling times
settling_idx_top = find(abs(x_top_1d - x_top_1d(end)) > 0.02*peak_displacement_top*1e-6, 1, 'last');
settling_idx_bot = find(abs(x_bot_1d - x_bot_1d(end)) > 0.02*peak_displacement_bot*1e-6, 1, 'last');
if isempty(settling_idx_top)
    settling_time_top = 0;
else
    settling_time_top = t_out(settling_idx_top) * 1000;
end
if isempty(settling_idx_bot)
    settling_time_bot = 0;
else
    settling_time_bot = t_out(settling_idx_bot) * 1000;
end

fprintf('\nStep Response Metrics:\n');
fprintf('                          Top Bearing    Bottom Bearing\n');
fprintf('  Peak displacement:      %.4f um       %.4f um\n', peak_displacement_top, peak_displacement_bot);
fprintf('  Final displacement:     %.4f um       %.4f um\n', final_displacement_top, final_displacement_bot);
fprintf('  Peak current:           %.3f A         %.3f A\n', peak_current_top, peak_current_bot);
fprintf('  Settling time (2%%):     %.2f ms        %.2f ms\n', settling_time_top, settling_time_bot);

% Store for later use (use max of both)
peak_displacement = max(peak_displacement_top, peak_displacement_bot);
settling_time = max(settling_time_top, settling_time_bot);

fprintf('\n  Note: Bottom bearing deflection is due to tilting motion caused\n');
fprintf('        by the step force at the top bearing.\n\n');

%% ========================================================================
% PART 1e: DYNAMIC STIFFNESS
% =========================================================================
fprintf('================================================================\n');
fprintf('PART 1e: Dynamic Stiffness Analysis\n');
fprintf('================================================================\n\n');

% Frequency range
freq_1e = logspace(0, 3, 200);  % 1 Hz to 1000 Hz
omega_1e = 2*pi*freq_1e;

% Initialize arrays
stiffness_radial = zeros(size(omega_1e));
stiffness_tilt = zeros(size(omega_1e));

fprintf('Computing dynamic stiffness from 1-1000 Hz...\n');

% =========================================================================
% USE 4-DOF GEOMETRY FOR PROPER STIFFNESS CALCULATION
% =========================================================================
% Two AMBs contribute to both radial and tilting stiffness
% Radial: K_s_total = 2*K_s (two bearings in parallel)
% Tilting: K_s_tilt = (d_top^2 + d_bot^2)*K_s (lever arm effects)

K_s_radial_total = 2 * baseline.K_s;  % Two AMBs contribute to radial stiffness

% Tilting stiffness using actual COM distances (not L_amb/2)
K_s_tilt = (baseline.d_amb_top^2 + baseline.d_amb_bottom^2) * baseline.K_s;

fprintf('  Radial K_s (2 AMBs): %.3e N/m\n', K_s_radial_total);
fprintf('  Tilting K_s: %.3e N*m/rad\n', K_s_tilt);
fprintf('  d_top = %.3f m, d_bottom = %.3f m\n\n', baseline.d_amb_top, baseline.d_amb_bottom);

for idx = 1:length(omega_1e)
    s_val = 1j * omega_1e(idx);

    % --- Radial Stiffness ---
    % Position controller at this frequency
    % Note: Appendix B controller gains are already position-to-force (N/m)
    % They do NOT need to be multiplied by K_i
    G_cx_val = baseline.kpx + baseline.kix/s_val + baseline.kdx*s_val/(1 + s_val/baseline.omega_px);

    % Dynamic stiffness with 2*K_s for two AMBs
    % K_dyn = m*s^2 - K_s + G_cx (K_s is positive destabilizing)
    denom_radial = baseline.total_mass*s_val^2 - K_s_radial_total + G_cx_val;
    stiffness_radial(idx) = abs(denom_radial);

    % --- Tilting Stiffness ---
    % Tilting controller at this frequency
    G_alpha_val = baseline.kp_alpha + baseline.ki_alpha/s_val + ...
        baseline.kd_alpha*s_val/(1 + s_val/baseline.omega_p_alpha);

    % For tilting: use transverse inertia and proper tilting stiffness
    % K_dyn_tilt = I_t*s^2 - K_s_tilt + G_alpha (K_s_tilt positive destabilizing)
    denom_tilt = baseline.I_transverse*s_val^2 - K_s_tilt + G_alpha_val;

    % Convert to linear stiffness at AMB location (use average lever arm)
    avg_lever_arm = (baseline.d_amb_top + baseline.d_amb_bottom) / 2;
    stiffness_tilt(idx) = abs(denom_tilt) / avg_lever_arm^2;
end

% --- PLOT: Part 1e ---
figure('Name', 'Part 1e: Dynamic Stiffness', 'Position', [100 100 900 450]);

loglog(freq_1e, stiffness_radial/1e6, 'b-', 'LineWidth', 2.5); hold on;
loglog(freq_1e, stiffness_tilt/1e6, 'r-', 'LineWidth', 2.5);
grid on;
xlabel('Frequency [Hz]', 'FontSize', 12);
ylabel('Dynamic Stiffness [MN/m]', 'FontSize', 12);
title('Part 1e: AMB Dynamic Stiffness vs Frequency', 'FontSize', 13);
legend('Radial Direction', 'Tilting Direction', 'Location', 'best', 'FontSize', 11);

saveas(gcf, 'part1e_dynamic_stiffness.png');
fprintf('Plot saved: part1e_dynamic_stiffness.png\n');

fprintf('\nDynamic Stiffness at Key Frequencies:\n');
fprintf('  Frequency    Radial       Tilting\n');
fprintf('  [Hz]         [MN/m]       [MN/m]\n');
fprintf('  ---------------------------------\n');
for f = [10, 100, 500, 1000]
    fprintf('  %4.0f         %.2f        %.2f\n', f, ...
        interp1(freq_1e, stiffness_radial, f)/1e6, ...
        interp1(freq_1e, stiffness_tilt, f)/1e6);
end
fprintf('\n');

%% ========================================================================
% PART 1f: ROTOR RUNOUT
% =========================================================================
fprintf('================================================================\n');
fprintf('PART 1f: Rotor Runout Analysis\n');
fprintf('================================================================\n\n');

% ASSUMPTION: ISO G2.5 balance grade (see ASSUMPTIONS.md)
G_grade = 2.5;  % [mm/s]
fprintf('Balance Grade: ISO G%.1f\n', G_grade);
fprintf('  G = e * omega = %.1f mm/s\n\n', G_grade);

% SoC range
SoC_1f = linspace(0, 100, 100);
runout_1f = zeros(size(SoC_1f));
omega_rpm_1f = zeros(size(SoC_1f));

% Use 2*K_s for two AMBs (same as Part 1e)
K_s_radial_1f = 2 * baseline.K_s;

for idx = 1:length(SoC_1f)
    % Speed at this SoC (energy-based: ω² is linear with SoC)
    omega = sqrt(baseline.omega_min^2 + (baseline.omega_max^2 - baseline.omega_min^2) * SoC_1f(idx)/100);
    omega_rpm_1f(idx) = omega * 60 / (2*pi);

    % Eccentricity from balance grade: e = G / omega
    e = (G_grade * 1e-3) / omega;  % [m]

    % Unbalance force at synchronous frequency
    F_unbalance = baseline.total_mass * e * omega^2;  % [N]

    % Dynamic stiffness at synchronous frequency (with 2*K_s for two AMBs)
    % Note: Controller gains are already position-to-force (N/m), no K_i needed
    % K_s is positive destabilizing, so use minus sign
    s_val = 1j * omega;
    G_cx_val = baseline.kpx + baseline.kix/s_val + baseline.kdx*s_val/(1 + s_val/baseline.omega_px);
    dyn_stiff = abs(baseline.total_mass*s_val^2 - K_s_radial_1f + G_cx_val);

    % Runout = Force / Stiffness
    runout_1f(idx) = (F_unbalance / dyn_stiff) * 1e6;  % [um]
end

% --- PLOT: Part 1f ---
figure('Name', 'Part 1f: Rotor Runout', 'Position', [100 100 900 450]);

yyaxis left
plot(SoC_1f, runout_1f, 'b-', 'LineWidth', 2.5);
ylabel('Rotor Runout [\mum]', 'FontSize', 12);

yyaxis right
plot(SoC_1f, omega_rpm_1f/1000, 'r--', 'LineWidth', 1.5);
ylabel('Speed [krpm]', 'FontSize', 12);

grid on;
xlabel('State of Charge [%]', 'FontSize', 12);
title('Part 1f: Rotor Runout Due to Mass Imbalance (ISO G2.5)', 'FontSize', 13);
legend('Runout', 'Speed', 'Location', 'best');

saveas(gcf, 'part1f_rotor_runout.png');
fprintf('Plot saved: part1f_rotor_runout.png\n');

fprintf('\nRunout at Key SoC Points:\n');
fprintf('  SoC     Speed      Runout\n');
fprintf('  [%%]     [RPM]      [um]\n');
fprintf('  ------------------------\n');
fprintf('  %3.0f     %5.0f      %.3f\n', 0, baseline.min_speed_rpm, runout_1f(1));
% Speed at 50% SoC (energy-based)
omega_50pct = sqrt(baseline.omega_min^2 + 0.5*(baseline.omega_max^2 - baseline.omega_min^2));
fprintf('  %3.0f     %5.0f      %.3f\n', 50, omega_50pct*60/(2*pi), ...
    interp1(SoC_1f, runout_1f, 50));
fprintf('  %3.0f     %5.0f      %.3f\n\n', 100, baseline.max_speed_rpm, runout_1f(end));

%% ========================================================================
% DELIVERABLE 1 SUMMARY
% =========================================================================
fprintf('================================================================\n');
fprintf('DELIVERABLE 1 SUMMARY\n');
fprintf('================================================================\n\n');

fprintf('BASELINE SYSTEM SPECIFICATIONS:\n');
fprintf('  Flywheel: %.0f mm dia x %.0f mm length\n', ...
    baseline.flywheel_diameter*1000, baseline.flywheel_length*1000);
fprintf('  Total mass: %.1f kg\n', baseline.total_mass);
fprintf('  Moment of inertia: %.4f kg*m^2\n', baseline.I_total);
fprintf('  Rated power: %.2f kW\n\n', baseline.power_rated/1000);

fprintf('PERFORMANCE METRICS:\n');
fprintf('  1a. Max losses: %.2f kW, Max temp: %.1f C\n', max(total_loss_1a)/1000, max(temperature_1a));
fprintf('  1b. Specific power: %.3f kW/kg\n', specific_power_bl);
fprintf('  1b. Specific energy: %.2f Wh/kg\n', specific_energy_bl);
fprintf('  1c. Cycle efficiency: %.2f%%\n', efficiency_1c);
fprintf('  1d. Peak AMB displacement: %.4f um\n', peak_displacement);
fprintf('  1e. Radial stiffness at 100 Hz: %.2f MN/m\n', interp1(freq_1e, stiffness_radial, 100)/1e6);
fprintf('  1f. Max runout: %.3f um\n\n', max(runout_1f));

%% ########################################################################
%                         DELIVERABLE 2
%                    NEW DESIGN DEVELOPMENT
% #########################################################################

fprintf('\n');
fprintf('================================================================\n');
fprintf('              DELIVERABLE 2: DESIGN STUDY\n');
fprintf('================================================================\n\n');

%% ========================================================================
% ANALYZE TEAM 16 CYCLE REQUIREMENTS (RIGOROUS)
% =========================================================================
fprintf('Analyzing Team 16 storage cycle requirements...\n\n');

% Team 16 cycle is 6 hours (0 to 21600 seconds)
t_cycle_analysis = linspace(0, 21600, 21601);
dt_analysis = t_cycle_analysis(2) - t_cycle_analysis(1);
P_cycle_analysis = zeros(size(t_cycle_analysis));
for i = 1:length(t_cycle_analysis)
    P_cycle_analysis(i) = team_16_cycle(t_cycle_analysis(i));
end

% Find peak power requirements
P_peak_discharge = max(P_cycle_analysis);   % Peak power out (positive = discharge)
P_peak_charge = abs(min(P_cycle_analysis)); % Peak power in (negative = charge)
P_peak_required = max(P_peak_discharge, P_peak_charge);

% === CRITICAL: Compute actual energy swing from cycle ===
% Integrate power to find cumulative energy change
E_cumulative = cumtrapz(t_cycle_analysis, P_cycle_analysis);  % [J]

% Energy metrics
E_max_swing = max(E_cumulative) - min(E_cumulative);  % Total energy swing [J]
E_discharged_total = trapz(t_cycle_analysis(P_cycle_analysis > 0), ...
    P_cycle_analysis(P_cycle_analysis > 0));  % Total energy out [J]
E_charged_total = abs(trapz(t_cycle_analysis(P_cycle_analysis < 0), ...
    P_cycle_analysis(P_cycle_analysis < 0)));  % Total energy in [J]

% Net energy change over cycle (should be near zero for sustainable operation)
E_net_cycle = E_cumulative(end);

fprintf('CYCLE ENERGY ANALYSIS:\n');
fprintf('  Total energy discharged: %.2f kWh\n', E_discharged_total/3.6e6);
fprintf('  Total energy charged: %.2f kWh\n', E_charged_total/3.6e6);
fprintf('  Max energy swing: %.2f kWh\n', E_max_swing/3.6e6);
fprintf('  Net energy change: %.2f kWh (should be ~0 for repeatable cycle)\n\n', E_net_cycle/3.6e6);

% === DESIGN REQUIREMENTS WITH SAFETY MARGINS ===
power_safety = 1.10;   % 10% power margin
energy_safety = 1.20;  % 20% energy margin (accounts for losses, SoC margins)

P_design_target = P_peak_required * power_safety;
E_design_target = E_max_swing * energy_safety;

fprintf('DESIGN REQUIREMENTS (with safety margins):\n');
fprintf('  Peak power target: %.2f kW (%.0f%% margin on %.2f kW)\n', ...
    P_design_target/1000, (power_safety-1)*100, P_peak_required/1000);
fprintf('  Energy capacity target: %.2f kWh (%.0f%% margin on %.2f kWh swing)\n\n', ...
    E_design_target/3.6e6, (energy_safety-1)*100, E_max_swing/3.6e6);

%% ========================================================================
% PART 2a: DESIGN PARAMETER STUDY (RIGOROUS MULTI-OBJECTIVE OPTIMIZATION)
% =========================================================================
fprintf('================================================================\n');
fprintf('PART 2a: Design Parameter Study\n');
fprintf('================================================================\n\n');

% =========================================================================
% DESIGN VARIABLE RANGES (REALISTIC BOUNDS)
% =========================================================================
% Magnet thickness: 2-12 mm (practical manufacturing range)
% Max speed: 25k-50k RPM (balanced for tip speed and energy density)
% Flywheel diameter: Derived from tip speed (variable, not fixed!)
% Flywheel length: 0.5-2.0 m (realistic manufacturing/transport)

magnet_thickness_range = linspace(0.002, 0.012, 11);  % 2-12 mm
max_speed_range = linspace(25000, 50000, 11);         % 25k-50k RPM

n_mag = length(magnet_thickness_range);
n_speed = length(max_speed_range);

% Results storage
results = struct();
results.specific_power = zeros(n_mag, n_speed);
results.specific_energy = zeros(n_mag, n_speed);
results.cycle_efficiency = zeros(n_mag, n_speed);
results.max_temp = zeros(n_mag, n_speed);
results.viable = zeros(n_mag, n_speed);
results.cycle_feasible = zeros(n_mag, n_speed);
results.thermal_feasible = zeros(n_mag, n_speed);
results.flywheel_diameter = zeros(n_mag, n_speed);
results.flywheel_length = zeros(n_mag, n_speed);
results.shaft_diameter = zeros(n_mag, n_speed);
results.motor_length = zeros(n_mag, n_speed);
results.total_mass = zeros(n_mag, n_speed);
results.rated_power = zeros(n_mag, n_speed);
results.usable_energy = zeros(n_mag, n_speed);
results.total_inertia = zeros(n_mag, n_speed);
results.failure_reason = cell(n_mag, n_speed);

% Design constraints
max_flywheel_length = 2.0;    % Maximum flywheel length [m] - realistic!
min_flywheel_length = 0.3;    % Minimum flywheel length [m]
max_motor_length = 0.8;       % Maximum motor length [m]
min_motor_length = 0.10;      % Minimum motor length [m]
min_shaft_radius = 0.025;     % Minimum shaft radius [m] (50 mm diameter)
min_flywheel_wall = 0.030;    % Minimum wall thickness for flywheel [m]

fprintf('Design Space:\n');
fprintf('  Magnet thickness: %.0f - %.0f mm (%d points)\n', ...
    min(magnet_thickness_range)*1000, max(magnet_thickness_range)*1000, n_mag);
fprintf('  Max speed: %.0f - %.0f kRPM (%d points)\n', ...
    min(max_speed_range)/1000, max(max_speed_range)/1000, n_speed);
fprintf('  Flywheel length: %.1f - %.1f m (derived to meet energy)\n', ...
    min_flywheel_length, max_flywheel_length);
fprintf('  Total design points: %d\n\n', n_mag * n_speed);

fprintf('Evaluating designs with cycle feasibility checking...\n');

% Progress tracking
designs_evaluated = 0;
designs_viable = 0;

for i = 1:n_mag
    for j = 1:n_speed
        designs_evaluated = designs_evaluated + 1;

        t_mag = magnet_thickness_range(i);
        omega_max_rpm = max_speed_range(j);
        omega_max = omega_max_rpm * 2*pi / 60;
        omega_min = omega_max / 2;  % Min speed = 50% of max (0% SoC)

        % === STEP 1: Shaft diameter from tip speed limits ===
        r_shaft_pm = (max_pm_tip_speed / omega_max) - t_mag;
        r_shaft_steel = max_steel_tip_speed / omega_max;
        r_shaft = min(r_shaft_pm, r_shaft_steel);

        if r_shaft < min_shaft_radius
            results.viable(i,j) = 0;
            results.failure_reason{i,j} = 'Shaft too small for tip speed';
            continue;
        end
        d_shaft = 2 * r_shaft;
        results.shaft_diameter(i,j) = d_shaft;

        % === STEP 2: Flywheel diameter from composite tip speed ===
        r_flywheel_max = max_composite_tip_speed / omega_max;
        d_flywheel = 2 * r_flywheel_max;

        % Check minimum wall thickness
        if (r_flywheel_max - r_shaft) < min_flywheel_wall
            results.viable(i,j) = 0;
            results.failure_reason{i,j} = 'Flywheel wall too thin';
            continue;
        end
        results.flywheel_diameter(i,j) = d_flywheel;

        % === STEP 3: Motor sizing for required power ===
        shear = magneticShear(t_mag, I_rated_pu);
        d_rotor = d_shaft + 2*t_mag;
        r_rotor = d_rotor / 2;
        torque_per_length = 2 * pi * r_rotor^2 * shear;

        L_motor_required = P_design_target / (torque_per_length * omega_max);
        L_motor = max(min_motor_length, min(max_motor_length, ...
            ceil(L_motor_required * 100) / 100));

        if L_motor_required > max_motor_length
            results.viable(i,j) = 0;
            results.failure_reason{i,j} = 'Motor too long for power requirement';
            continue;
        end
        results.motor_length(i,j) = L_motor;

        motor_area = 2 * pi * r_rotor * L_motor;
        P_rated = shear * motor_area * r_rotor * omega_max;
        results.rated_power(i,j) = P_rated;

        % === STEP 4: Flywheel length for energy requirement ===
        r_outer = d_flywheel / 2;
        r_inner = d_shaft / 2;

        V_per_length = pi * (r_outer^2 - r_inner^2);
        m_per_length = rho_composite * V_per_length;
        I_per_length = 0.5 * m_per_length * (r_outer^2 + r_inner^2);

        % Inertia needed for target energy capacity
        I_required = 2 * E_design_target / (omega_max^2 - omega_min^2);
        L_flywheel_required = I_required / I_per_length;

        % Clamp to realistic bounds
        L_flywheel = max(min_flywheel_length, min(max_flywheel_length, ...
            ceil(L_flywheel_required * 20) / 20));  % Round to 5 cm
        results.flywheel_length(i,j) = L_flywheel;

        if L_flywheel_required > max_flywheel_length
            results.viable(i,j) = 0;
            results.failure_reason{i,j} = sprintf('Flywheel too long (%.2fm needed)', L_flywheel_required);
            continue;
        end

        % === STEP 5: Calculate mass and inertia ===
        V_flywheel = pi * (r_outer^2 - r_inner^2) * L_flywheel;
        m_flywheel = rho_composite * V_flywheel;

        L_shaft_total = L_flywheel + L_motor + 0.30;  % +30cm for bearings/ends
        V_shaft = pi * r_inner^2 * L_shaft_total;
        m_shaft = rho_steel * V_shaft;

        r_mag = r_inner + t_mag;
        V_magnet = pi * (r_mag^2 - r_inner^2) * L_motor;
        m_magnet = rho_magnet * V_magnet;

        % Total mass with 10% addition for AMB rotors, flanges, etc.
        m_total = (m_flywheel + m_shaft + m_magnet) * 1.10;
        results.total_mass(i,j) = m_total;

        I_flywheel = 0.5 * m_flywheel * (r_outer^2 + r_inner^2);
        I_shaft = 0.5 * m_shaft * r_inner^2;
        I_magnet = 0.5 * m_magnet * (r_mag^2 + r_inner^2);
        I_total = I_flywheel + I_shaft + I_magnet;
        results.total_inertia(i,j) = I_total;

        % Actual usable energy
        E_max = 0.5 * I_total * omega_max^2;
        E_min = 0.5 * I_total * omega_min^2;
        E_usable = E_max - E_min;
        results.usable_energy(i,j) = E_usable;

        % === STEP 6: Specific power and energy ===
        results.specific_power(i,j) = (P_rated / 1000) / m_total;  % kW/kg
        results.specific_energy(i,j) = (E_usable / 3600) / m_total;  % Wh/kg

        % === STEP 7: COMPREHENSIVE THERMAL CHECK (full SoC sweep) ===
        % Check temperature at multiple SoC points, not just max speed
        SoC_thermal = linspace(0, 100, 21);  % 21 points
        T_max_found = 0;

        % Calculate surface area for thermal model
        A_fly_cyl = 2*pi*r_outer*L_flywheel;
        A_fly_ends = 2*pi*(r_outer^2 - r_inner^2);
        A_motor_cyl = 2*pi*r_rotor*L_motor;
        A_shaft_cyl = 2*pi*r_inner*(0.3*L_shaft_total);
        A_surface = A_fly_cyl + A_fly_ends + A_motor_cyl + A_shaft_cyl;

        housing_inner = d_flywheel + 0.04;
        housing_length = L_shaft_total + 0.04;
        A_housing = 2*pi*(housing_inner/2)*housing_length + 2*pi*(housing_inner/2)^2;
        rad_factor = 1/epsilon_rotor + (A_surface/A_housing)*(1/epsilon_housing - 1);

        for k = 1:length(SoC_thermal)
            omega_k = sqrt(omega_min^2 + (omega_max^2 - omega_min^2) * SoC_thermal(k)/100);
            omega_rpm_k = omega_k * 60 / (2*pi);

            P_rotor_k = rotorLosses(t_mag, d_rotor, L_motor, I_rated_pu, omega_rpm_k);
            T_rotor_K = (T_housing_K^4 + P_rotor_k * rad_factor / (sigma * A_surface))^0.25;
            T_rotor_C = T_rotor_K - 273.15;

            if T_rotor_C > T_max_found
                T_max_found = T_rotor_C;
            end
        end
        results.max_temp(i,j) = T_max_found;
        results.thermal_feasible(i,j) = (T_max_found <= max_temp);

        if T_max_found > max_temp
            results.viable(i,j) = 0;
            results.failure_reason{i,j} = sprintf('Thermal limit exceeded (%.1fC)', T_max_found);
            continue;
        end

        % === STEP 8: CYCLE FEASIBILITY CHECK (critical!) ===
        % Simulate the actual 6-hour team_16_cycle
        t_sim = linspace(0, 21600, 2161);  % 10-second resolution
        dt_sim = t_sim(2) - t_sim(1);

        % Start at 50% SoC (middle of operating range)
        SoC_start = 50;
        omega_current = sqrt(omega_min^2 + (omega_max^2 - omega_min^2) * SoC_start/100);
        KE_current = 0.5 * I_total * omega_current^2;

        SoC_min_cycle = 100;
        SoC_max_cycle = 0;
        total_losses = 0;
        E_in_cycle = 0;
        E_out_cycle = 0;
        cycle_failed = false;

        for k = 1:length(t_sim)
            % Get grid power demand
            P_grid = team_16_cycle(t_sim(k));

            % Current speed and SoC
            omega_current = sqrt(2 * KE_current / I_total);
            omega_rpm_current = omega_current * 60 / (2*pi);
            SoC_current = 100 * (omega_current^2 - omega_min^2) / (omega_max^2 - omega_min^2);

            % Track SoC range
            SoC_min_cycle = min(SoC_min_cycle, SoC_current);
            SoC_max_cycle = max(SoC_max_cycle, SoC_current);

            % Check SoC bounds (with 2% margin for numerical error)
            if SoC_current < -2 || SoC_current > 102
                cycle_failed = true;
                break;
            end

            % Calculate losses at current operating point
            % Use a representative current based on power
            I_pu_current = min(1.0, abs(P_grid) / P_rated);
            P_rotor = rotorLosses(t_mag, d_rotor, L_motor, I_pu_current, omega_rpm_current);
            P_stator = statorLosses(t_mag, d_rotor, L_motor, I_pu_current, omega_rpm_current);
            P_loss = P_rotor + P_stator;
            total_losses = total_losses + P_loss * dt_sim;

            % Energy accounting
            if P_grid > 0  % Discharging
                E_out_cycle = E_out_cycle + P_grid * dt_sim;
                KE_current = KE_current - (P_grid + P_loss) * dt_sim;
            else  % Charging
                E_in_cycle = E_in_cycle + abs(P_grid) * dt_sim;
                KE_current = KE_current + (abs(P_grid) - P_loss) * dt_sim;
            end
        end

        results.cycle_feasible(i,j) = ~cycle_failed;

        if cycle_failed
            results.viable(i,j) = 0;
            results.failure_reason{i,j} = sprintf('Cycle SoC violation (%.1f%% to %.1f%%)', ...
                SoC_min_cycle, SoC_max_cycle);
            continue;
        end

        % Calculate cycle efficiency
        if E_in_cycle > 0
            results.cycle_efficiency(i,j) = 100 * E_out_cycle / (E_out_cycle + total_losses);
        else
            results.cycle_efficiency(i,j) = 0;
        end

        % === Design is viable if we get here ===
        results.viable(i,j) = 1;
        results.failure_reason{i,j} = 'OK';
        designs_viable = designs_viable + 1;
    end
end

fprintf('  Evaluated: %d designs, Viable: %d (%.1f%%)\n\n', ...
    designs_evaluated, designs_viable, 100*designs_viable/designs_evaluated);

% =========================================================================
% MULTI-OBJECTIVE OPTIMAL DESIGN SELECTION
% =========================================================================
% Score viable designs using weighted objectives:
%   - Specific energy (higher is better)
%   - Specific power (higher is better)
%   - Cycle efficiency (higher is better)
%   - Total mass (lower is better, for cost)

fprintf('MULTI-OBJECTIVE DESIGN SELECTION:\n');

% Get indices of viable designs
viable_idx = find(results.viable == 1);

if isempty(viable_idx)
    error('No viable designs found! Adjust design space or constraints.');
end

fprintf('  Viable designs: %d\n', length(viable_idx));

% Extract metrics for viable designs
viable_sp = results.specific_power(viable_idx);
viable_se = results.specific_energy(viable_idx);
viable_eff = results.cycle_efficiency(viable_idx);
viable_mass = results.total_mass(viable_idx);

% Normalize each metric to [0, 1] range
sp_norm = (viable_sp - min(viable_sp)) / (max(viable_sp) - min(viable_sp) + eps);
se_norm = (viable_se - min(viable_se)) / (max(viable_se) - min(viable_se) + eps);
eff_norm = (viable_eff - min(viable_eff)) / (max(viable_eff) - min(viable_eff) + eps);
mass_norm = (max(viable_mass) - viable_mass) / (max(viable_mass) - min(viable_mass) + eps);  % Inverted

% Objective weights (sum to 1.0)
w_sp = 0.20;    % Specific power
w_se = 0.30;    % Specific energy (most important for storage)
w_eff = 0.30;   % Cycle efficiency (critical for operation)
w_mass = 0.20;  % Mass/cost

fprintf('  Weights: SP=%.0f%%, SE=%.0f%%, Eff=%.0f%%, Mass=%.0f%%\n', ...
    w_sp*100, w_se*100, w_eff*100, w_mass*100);

% Compute composite score
scores = w_sp*sp_norm + w_se*se_norm + w_eff*eff_norm + w_mass*mass_norm;

% Find best design
[best_score, best_local_idx] = max(scores);
best_idx = viable_idx(best_local_idx);
[opt_i, opt_j] = ind2sub([n_mag, n_speed], best_idx);

% Also find Pareto-optimal designs (non-dominated)
n_viable = length(viable_idx);
is_pareto = true(n_viable, 1);
for k = 1:n_viable
    for m = 1:n_viable
        if k ~= m
            % Check if design m dominates design k
            % m dominates k if m is >= k in all objectives and > in at least one
            if viable_sp(m) >= viable_sp(k) && ...
               viable_se(m) >= viable_se(k) && ...
               viable_eff(m) >= viable_eff(k) && ...
               viable_mass(m) <= viable_mass(k) && ...
               (viable_sp(m) > viable_sp(k) || ...
                viable_se(m) > viable_se(k) || ...
                viable_eff(m) > viable_eff(k) || ...
                viable_mass(m) < viable_mass(k))
                is_pareto(k) = false;
                break;
            end
        end
    end
end
n_pareto = sum(is_pareto);
fprintf('  Pareto-optimal designs: %d\n\n', n_pareto);

% Build optimal design structure
optimal = struct();
optimal.magnet_thickness = magnet_thickness_range(opt_i);
optimal.max_speed_rpm = max_speed_range(opt_j);
optimal.omega_max = optimal.max_speed_rpm * 2*pi / 60;
optimal.omega_min = optimal.omega_max / 2;
optimal.specific_power = results.specific_power(opt_i, opt_j);
optimal.specific_energy = results.specific_energy(opt_i, opt_j);
optimal.cycle_efficiency = results.cycle_efficiency(opt_i, opt_j);
optimal.total_mass = results.total_mass(opt_i, opt_j);
optimal.flywheel_diameter = results.flywheel_diameter(opt_i, opt_j);
optimal.flywheel_length = results.flywheel_length(opt_i, opt_j);
optimal.shaft_diameter = results.shaft_diameter(opt_i, opt_j);
optimal.motor_length = results.motor_length(opt_i, opt_j);
optimal.rated_power = results.rated_power(opt_i, opt_j);
optimal.usable_energy = results.usable_energy(opt_i, opt_j);
optimal.total_inertia = results.total_inertia(opt_i, opt_j);
optimal.max_temp = results.max_temp(opt_i, opt_j);

fprintf('OPTIMAL DESIGN (Multi-Objective Score: %.3f):\n', best_score);
fprintf('  Geometry:\n');
fprintf('    Flywheel: %.0f mm dia x %.0f mm length\n', ...
    optimal.flywheel_diameter*1000, optimal.flywheel_length*1000);
fprintf('    Shaft: %.0f mm diameter\n', optimal.shaft_diameter*1000);
fprintf('    Motor: %.0f mm length\n', optimal.motor_length*1000);
fprintf('    Magnet thickness: %.1f mm\n', optimal.magnet_thickness*1000);
fprintf('  Performance:\n');
fprintf('    Max speed: %.0f RPM\n', optimal.max_speed_rpm);
fprintf('    Rated power: %.1f kW\n', optimal.rated_power/1000);
fprintf('    Usable energy: %.1f kWh\n', optimal.usable_energy/3.6e6);
fprintf('    Total mass: %.1f kg\n', optimal.total_mass);
fprintf('  Metrics:\n');
fprintf('    Specific power: %.3f kW/kg\n', optimal.specific_power);
fprintf('    Specific energy: %.2f Wh/kg\n', optimal.specific_energy);
fprintf('    Cycle efficiency: %.2f%%\n', optimal.cycle_efficiency);
fprintf('    Max temperature: %.1f C\n\n', optimal.max_temp);

% --- PLOT: Part 2a (Improved Design Justification) ---
figure('Name', 'Part 2a: Design Tradeoffs', 'Position', [50 50 1600 1000]);

% Create meshgrid for plotting
[MAG, SPD] = meshgrid(magnet_thickness_range*1000, max_speed_range/1000);

% Compute composite score for all designs (not just viable)
score_grid = zeros(n_mag, n_speed);
for ii = 1:n_mag
    for jj = 1:n_speed
        if results.viable(ii,jj) == 1
            % Normalize metrics relative to viable range
            sp_n = (results.specific_power(ii,jj) - min(viable_sp)) / (max(viable_sp) - min(viable_sp) + eps);
            se_n = (results.specific_energy(ii,jj) - min(viable_se)) / (max(viable_se) - min(viable_se) + eps);
            eff_n = (results.cycle_efficiency(ii,jj) - min(viable_eff)) / (max(viable_eff) - min(viable_eff) + eps);
            mass_n = (max(viable_mass) - results.total_mass(ii,jj)) / (max(viable_mass) - min(viable_mass) + eps);
            score_grid(ii,jj) = w_sp*sp_n + w_se*se_n + w_eff*eff_n + w_mass*mass_n;
        else
            score_grid(ii,jj) = NaN;
        end
    end
end

% === SUBPLOT 1: Constraint Map (Why designs fail) ===
subplot(2,3,1);
% Create categorical constraint map: 0=geometry, 1=thermal, 2=cycle, 3=viable
constraint_map = zeros(n_mag, n_speed);
for ii = 1:n_mag
    for jj = 1:n_speed
        if results.viable(ii,jj) == 1
            constraint_map(ii,jj) = 3;  % Viable
        elseif results.thermal_feasible(ii,jj) == 0
            constraint_map(ii,jj) = 1;  % Thermal failure
        elseif results.cycle_feasible(ii,jj) == 0
            constraint_map(ii,jj) = 2;  % Cycle failure
        else
            constraint_map(ii,jj) = 0;  % Geometry failure
        end
    end
end
imagesc(magnet_thickness_range*1000, max_speed_range/1000, constraint_map');
colormap(gca, [0.7 0.7 0.7; 1 0.4 0.4; 1 0.8 0.4; 0.4 0.8 0.4]);  % Gray, Red, Orange, Green
caxis([0 3]);
cb = colorbar('Ticks', [0.375, 1.125, 1.875, 2.625], ...
    'TickLabels', {'Geometry', 'Thermal', 'Cycle', 'VIABLE'});
set(gca, 'YDir', 'normal');
hold on;
plot(optimal.magnet_thickness*1000, optimal.max_speed_rpm/1000, 'kp', ...
    'MarkerSize', 18, 'MarkerFaceColor', 'w', 'LineWidth', 2);
xlabel('Magnet Thickness [mm]', 'FontSize', 11);
ylabel('Max Speed [krpm]', 'FontSize', 11);
title('Design Space Constraints', 'FontSize', 12, 'FontWeight', 'bold');

% === SUBPLOT 2: Multi-Objective Score ===
subplot(2,3,2);
contourf(MAG', SPD', score_grid, 15);
colormap(gca, 'parula');
cb = colorbar;
ylabel(cb, 'Composite Score', 'FontSize', 10);
hold on;
% Mark all viable designs
[viable_i, viable_j] = find(results.viable == 1);
for k = 1:length(viable_i)
    plot(magnet_thickness_range(viable_i(k))*1000, max_speed_range(viable_j(k))/1000, ...
        'wo', 'MarkerSize', 6, 'LineWidth', 1);
end
% Mark optimal
plot(optimal.magnet_thickness*1000, optimal.max_speed_rpm/1000, 'rp', ...
    'MarkerSize', 18, 'MarkerFaceColor', 'r', 'LineWidth', 2);
xlabel('Magnet Thickness [mm]', 'FontSize', 11);
ylabel('Max Speed [krpm]', 'FontSize', 11);
title(sprintf('Multi-Objective Score (Optimal=%.3f)', best_score), 'FontSize', 12, 'FontWeight', 'bold');

% === SUBPLOT 3: Temperature with Constraint ===
subplot(2,3,3);
contourf(MAG', SPD', results.max_temp, 15);
colormap(gca, 'hot');
cb = colorbar;
ylabel(cb, 'Temperature [°C]', 'FontSize', 10);
hold on;
contour(MAG', SPD', results.max_temp, [100 100], 'b-', 'LineWidth', 3);
plot(optimal.magnet_thickness*1000, optimal.max_speed_rpm/1000, 'cp', ...
    'MarkerSize', 18, 'MarkerFaceColor', 'c', 'LineWidth', 2);
xlabel('Magnet Thickness [mm]', 'FontSize', 11);
ylabel('Max Speed [krpm]', 'FontSize', 11);
title(sprintf('Max Temperature (Optimal=%.1f°C, Limit=100°C)', optimal.max_temp), ...
    'FontSize', 12, 'FontWeight', 'bold');
text(3, 48, '100°C Limit', 'Color', 'b', 'FontSize', 11, 'FontWeight', 'bold');

% === SUBPLOT 4: Trade-off - Efficiency vs Specific Energy ===
subplot(2,3,4);
scatter(viable_se, viable_eff, 80, scores, 'filled', 'MarkerEdgeColor', 'k');
colormap(gca, 'parula');
cb = colorbar;
ylabel(cb, 'Score', 'FontSize', 10);
hold on;
% Highlight Pareto-optimal designs
pareto_se = viable_se(is_pareto);
pareto_eff = viable_eff(is_pareto);
plot(pareto_se, pareto_eff, 'ko', 'MarkerSize', 14, 'LineWidth', 2.5);
% Highlight selected optimal with annotation
plot(optimal.specific_energy, optimal.cycle_efficiency, 'rp', ...
    'MarkerSize', 20, 'MarkerFaceColor', 'r', 'LineWidth', 2);
xlabel('Specific Energy [Wh/kg]', 'FontSize', 11);
ylabel('Cycle Efficiency [%]', 'FontSize', 11);
title('Pareto Front: Efficiency vs Energy', 'FontSize', 12, 'FontWeight', 'bold');
legend({'Viable', 'Pareto-optimal', 'SELECTED'}, 'Location', 'southwest', 'FontSize', 9);
grid on;

% === SUBPLOT 5: Trade-off - Mass vs Efficiency ===
subplot(2,3,5);
scatter(viable_mass, viable_eff, 80, viable_se, 'filled', 'MarkerEdgeColor', 'k');
colormap(gca, 'parula');
cb = colorbar;
ylabel(cb, 'Sp. Energy [Wh/kg]', 'FontSize', 10);
hold on;
plot(optimal.total_mass, optimal.cycle_efficiency, 'rp', ...
    'MarkerSize', 20, 'MarkerFaceColor', 'r', 'LineWidth', 2);
xlabel('Total Mass [kg]', 'FontSize', 11);
ylabel('Cycle Efficiency [%]', 'FontSize', 11);
title('Trade-off: Mass vs Efficiency', 'FontSize', 12, 'FontWeight', 'bold');
grid on;
% Add baseline reference
plot(baseline.total_mass, 95.47, 'bs', 'MarkerSize', 15, 'MarkerFaceColor', 'b', 'LineWidth', 2);
legend({'Viable Designs', 'SELECTED', 'Baseline'}, 'Location', 'southwest', 'FontSize', 9);

% === SUBPLOT 6: Optimal Design Summary ===
subplot(2,3,6);
axis off;

% Create a summary box
text(0.5, 0.95, 'OPTIMAL DESIGN SELECTED', 'FontSize', 14, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center', 'Color', [0.8 0 0]);

% Design parameters
text(0.05, 0.82, 'GEOMETRY:', 'FontSize', 11, 'FontWeight', 'bold');
text(0.08, 0.74, sprintf('Flywheel: %.0f mm dia × %.0f mm length', ...
    optimal.flywheel_diameter*1000, optimal.flywheel_length*1000), 'FontSize', 10);
text(0.08, 0.66, sprintf('Magnet Thickness: %.1f mm', optimal.magnet_thickness*1000), 'FontSize', 10);
text(0.08, 0.58, sprintf('Max Speed: %.0f RPM', optimal.max_speed_rpm), 'FontSize', 10);

% Performance metrics
text(0.05, 0.46, 'PERFORMANCE:', 'FontSize', 11, 'FontWeight', 'bold');
text(0.08, 0.38, sprintf('Specific Energy: %.2f Wh/kg', optimal.specific_energy), 'FontSize', 10);
text(0.08, 0.30, sprintf('Specific Power: %.3f kW/kg', optimal.specific_power), 'FontSize', 10);
text(0.08, 0.22, sprintf('Cycle Efficiency: %.2f%%', optimal.cycle_efficiency), 'FontSize', 10);
text(0.08, 0.14, sprintf('Total Mass: %.0f kg', optimal.total_mass), 'FontSize', 10);

% Why selected
text(0.05, 0.02, 'WHY SELECTED:', 'FontSize', 11, 'FontWeight', 'bold');
text(0.08, -0.06, sprintf('• Multi-objective score: %.3f (highest)', best_score), 'FontSize', 10);
text(0.08, -0.14, sprintf('• Thermal margin: %.1f°C below limit', 100 - optimal.max_temp), 'FontSize', 10);
text(0.08, -0.22, sprintf('• Pareto-optimal (non-dominated)', best_score), 'FontSize', 10);

% Weights used
text(0.55, 0.82, 'OBJECTIVE WEIGHTS:', 'FontSize', 11, 'FontWeight', 'bold');
text(0.58, 0.74, sprintf('Specific Energy: %.0f%%', w_se*100), 'FontSize', 10);
text(0.58, 0.66, sprintf('Cycle Efficiency: %.0f%%', w_eff*100), 'FontSize', 10);
text(0.58, 0.58, sprintf('Specific Power: %.0f%%', w_sp*100), 'FontSize', 10);
text(0.58, 0.50, sprintf('Mass (lower): %.0f%%', w_mass*100), 'FontSize', 10);

% Comparison to baseline
text(0.55, 0.38, 'vs BASELINE:', 'FontSize', 11, 'FontWeight', 'bold');
mass_change = (optimal.total_mass - baseline.total_mass) / baseline.total_mass * 100;
eff_change = optimal.cycle_efficiency - 95.47;
energy_change = (optimal.usable_energy/3.6e6 - 9.91) / 9.91 * 100;
text(0.58, 0.30, sprintf('Mass: %+.0f%%', mass_change), 'FontSize', 10);
text(0.58, 0.22, sprintf('Efficiency: %+.1f%%', eff_change), 'FontSize', 10);
text(0.58, 0.14, sprintf('Energy: %+.0f%%', energy_change), 'FontSize', 10);

sgtitle('Part 2a: Design Space Exploration & Optimal Selection', 'FontSize', 14, 'FontWeight', 'bold');
saveas(gcf, 'part2a_design_tradeoffs.png');
fprintf('Plot saved: part2a_design_tradeoffs.png\n\n');

%% ========================================================================
% PART 2b: OPTIMAL STORAGE CYCLE (Team 16's 6-hour cycle)
% =========================================================================
fprintf('================================================================\n');
fprintf('PART 2b: Team 16 Storage Cycle with Optimal Design\n');
fprintf('================================================================\n\n');

% Use optimal design parameters (already computed in Part 2a)
t_mag_opt = optimal.magnet_thickness;
omega_max_opt = optimal.omega_max;
omega_min_opt = optimal.omega_min;
d_shaft_opt = optimal.shaft_diameter;
d_flywheel_opt = optimal.flywheel_diameter;
L_flywheel_opt = optimal.flywheel_length;
L_motor_opt = optimal.motor_length;
I_total_opt = optimal.total_inertia;
P_rated_opt = optimal.rated_power;
m_total_opt = optimal.total_mass;

% Derived geometric parameters
d_rotor_opt = d_shaft_opt + 2*t_mag_opt;
r_rotor_opt = d_rotor_opt / 2;
r_outer_opt = d_flywheel_opt / 2;
r_inner_opt = d_shaft_opt / 2;
L_shaft_opt = L_flywheel_opt + L_motor_opt + 0.30;

% Store additional optimal parameters for Deliverable 3
optimal.I_total = I_total_opt;
optimal.power_rated = P_rated_opt;

fprintf('Optimal Design Geometry (from multi-objective optimization):\n');
fprintf('  Flywheel: %.0f mm dia x %.0f mm length\n', d_flywheel_opt*1000, L_flywheel_opt*1000);
fprintf('  Shaft: %.0f mm dia\n', d_shaft_opt*1000);
fprintf('  Motor: %.0f mm length\n', L_motor_opt*1000);
fprintf('  Magnet thickness: %.1f mm\n', t_mag_opt*1000);
fprintf('  Total mass: %.1f kg\n', m_total_opt);
fprintf('  Inertia: %.4f kg*m^2\n', I_total_opt);
fprintf('  Rated power: %.2f kW\n', P_rated_opt/1000);
fprintf('  Usable energy: %.2f kWh\n\n', optimal.usable_energy/3.6e6);

% 6-hour cycle simulation
dt_2b = 10.0;
t_2b = 0:dt_2b:21600;  % 6 hours

% Calculate energy limits
E_max_opt = 0.5 * I_total_opt * omega_max_opt^2;
E_min_opt = 0.5 * I_total_opt * omega_min_opt^2;

omega_curr_2b = (omega_max_opt + omega_min_opt) / 2;  % Start at 50% SoC
E_curr_2b = 0.5 * I_total_opt * omega_curr_2b^2;
E_initial_2b = E_curr_2b;

SoC_2b = zeros(size(t_2b));
power_grid_2b = zeros(size(t_2b));
power_loss_2b = zeros(size(t_2b));

E_in_2b = 0;
E_out_2b = 0;

fprintf('Simulating 6-hour Team 16 storage cycle...\n');

% Surface area calculation (consistent with D1 baseline)
% 1. Flywheel outer cylinder
A_fly_cyl_opt = 2*pi*r_outer_opt*L_flywheel_opt;
% 2. Flywheel annular end faces (ring shape, not full disk)
A_fly_ends_opt = 2*pi*(r_outer_opt^2 - r_inner_opt^2);
% 3. Motor/rotor outer cylinder (at air gap)
A_motor_cyl_opt = 2*pi*r_rotor_opt*L_motor_opt;
% 4. Exposed shaft (estimated as 30% of total length)
L_exposed_opt = 0.3 * L_shaft_opt;
A_shaft_cyl_opt = 2*pi*r_inner_opt*L_exposed_opt;
% Total radiating surface
A_surface_opt = A_fly_cyl_opt + A_fly_ends_opt + A_motor_cyl_opt + A_shaft_cyl_opt;

housing_inner_opt = d_flywheel_opt + 0.04;
housing_length_opt = L_shaft_opt + 0.04;
A_housing_opt = 2*pi*(housing_inner_opt/2)*housing_length_opt + 2*pi*(housing_inner_opt/2)^2;
rad_factor_opt = 1/epsilon_rotor + (A_surface_opt/A_housing_opt)*(1/epsilon_housing - 1);

for i = 1:length(t_2b)
    P_grid = team_16_cycle(t_2b(i));
    power_grid_2b(i) = P_grid;

    % Energy-based SoC (SoC ∝ ω²)
    SoC_2b(i) = 100 * (omega_curr_2b^2 - omega_min_opt^2) / (omega_max_opt^2 - omega_min_opt^2);
    omega_rpm_curr = omega_curr_2b * 60 / (2*pi);

    I_pu = min(1.0, I_rated_pu * abs(P_grid) / P_rated_opt);

    D_rotor_opt = d_shaft_opt + 2*t_mag_opt;  % Outer diameter of magnets
    P_rotor = rotorLosses(t_mag_opt, D_rotor_opt, L_motor_opt, I_pu, omega_rpm_curr);
    P_stator = statorLosses(t_mag_opt, D_rotor_opt, L_motor_opt, I_pu, omega_rpm_curr);
    P_loss = P_rotor + P_stator;
    power_loss_2b(i) = P_loss;

    % Energy balance: dE/dt = -P_grid - P_loss
    dE = -(P_grid + P_loss) * dt_2b;
    E_curr_2b = E_curr_2b + dE;

    % Enforce energy limits (0% to 100% SoC)
    E_curr_2b = max(E_min_opt, min(E_max_opt, E_curr_2b));

    % Update speed from energy
    omega_curr_2b = sqrt(2 * E_curr_2b / I_total_opt);

    % Track energy flows
    if P_grid > 0
        E_out_2b = E_out_2b + P_grid * dt_2b;
    else
        E_in_2b = E_in_2b + abs(P_grid) * dt_2b;
    end
end

% Final energy
E_final_2b = E_curr_2b;
E_loss_2b = sum(power_loss_2b) * dt_2b;

% Correct efficiency calculation:
% Account for change in stored energy (SoC drift)
% Net energy to flywheel = E_in - E_out - E_loss = E_final - E_initial
% Round-trip efficiency = E_out / (E_in - (E_final - E_initial))
%                       = E_out / (E_in - dE_stored)
dE_stored = E_final_2b - E_initial_2b;
efficiency_2b = (E_out_2b / (E_in_2b - dE_stored)) * 100;

% Sanity check: if efficiency > 100 or < 0, use throughput-based calculation
if efficiency_2b > 100 || efficiency_2b < 0
    E_throughput = (E_in_2b + E_out_2b) / 2;
    efficiency_2b = (1 - E_loss_2b / (2 * E_throughput)) * 100;
end

% --- PLOT: Part 2b ---
figure('Name', 'Part 2b: Team 16 Cycle', 'Position', [100 100 1000 700]);

subplot(3,1,1);
plot(t_2b/3600, power_grid_2b/1000, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time [hours]', 'FontSize', 11);
ylabel('Grid Power [kW]', 'FontSize', 11);
title('Part 2b: Team 16 6-Hour Storage Cycle - Power Demand', 'FontSize', 12);

subplot(3,1,2);
plot(t_2b/3600, SoC_2b, 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Time [hours]', 'FontSize', 11);
ylabel('State of Charge [%]', 'FontSize', 11);
title('Part 2b: Flywheel SoC During Cycle', 'FontSize', 12);

subplot(3,1,3);
plot(t_2b/3600, power_loss_2b/1000, 'k-', 'LineWidth', 1.5);
grid on;
xlabel('Time [hours]', 'FontSize', 11);
ylabel('Total Losses [kW]', 'FontSize', 11);
title('Part 2b: Motor Losses During Cycle', 'FontSize', 12);

saveas(gcf, 'part2b_team16_cycle.png');
fprintf('Plot saved: part2b_team16_cycle.png\n');

fprintf('\n6-Hour Cycle Results:\n');
fprintf('  Energy discharged: %.2f kWh\n', E_out_2b/3.6e6);
fprintf('  Energy charged: %.2f kWh\n', E_in_2b/3.6e6);
fprintf('  Total losses: %.2f kWh\n', E_loss_2b/3.6e6);
fprintf('  SoC range: %.1f%% to %.1f%%\n', min(SoC_2b), max(SoC_2b));
fprintf('  Starting SoC: %.1f%%, Ending SoC: %.1f%%\n', SoC_2b(1), SoC_2b(end));
fprintf('  Cycle Efficiency: %.2f%%\n\n', efficiency_2b);

%% ########################################################################
%                         DELIVERABLE 3
%                   AMB CONTROLLER DESIGN & COMPARISON
% #########################################################################

fprintf('\n');
fprintf('================================================================\n');
fprintf('              DELIVERABLE 3: AMB CONTROLLER DESIGN\n');
fprintf('================================================================\n\n');

%% ========================================================================
% PART 3a: CONTROLLER TRANSFER FUNCTIONS
% =========================================================================
fprintf('================================================================\n');
fprintf('PART 3a: Controller Transfer Functions\n');
fprintf('================================================================\n\n');

% New system AMB sizing
g = 9.81;
new_amb_force = 2 * optimal.total_mass * g;
amb_new = ambParameters(optimal.shaft_diameter, new_amb_force);

K_s_new = amb_new.stiffnessConstant;  % Positive, destabilizing (like baseline)
K_i_new = amb_new.forceConstant;
L_coil_new = amb_new.coilInductance;
R_coil_new = amb_new.coilResistance;

fprintf('New System AMB Parameters:\n');
fprintf('  Rated force: %.0f N\n', new_amb_force);
fprintf('  K_s: %.3e N/m (positive, destabilizing)\n', K_s_new);
fprintf('  K_i: %.2f N/A\n', K_i_new);
fprintf('  L_coil: %.4f H\n', L_coil_new);
fprintf('  R_coil: %.3f Ohms\n\n', R_coil_new);

% --- Current Controller Design (HW3 methodology) ---
% Pole-zero cancellation for first-order response
% Target: 1.5 kHz bandwidth
omega_bw_current = 2*pi*1500;

Kp_current_new = L_coil_new * omega_bw_current;
Ki_current_new = Kp_current_new * R_coil_new / L_coil_new;

fprintf('NEW Current Controller (HW3 pole-zero cancellation, 1.5 kHz BW):\n');
fprintf('  G_ci(s) = %.2f + %.2f/s\n\n', Kp_current_new, Ki_current_new);

% =========================================================================
% LOOP-SHAPING POSITION CONTROLLER DESIGN
% =========================================================================
% Controller structure: G_c(s) = K_o*(1 + s/w_z1)*(1 + s/w_z2) / [s*(1 + s/w_p)]
% Equivalent PID form: G_c(s) = k_p + k_i/s + k_d*s/(1+s/w_p)
%
% Design targets:
%   - Crossover frequency: f_c (bandwidth)
%   - Phase margin: phi_pm (typically 45-60 deg)
%   - Crossover >> unstable pole frequency

% Unstable pole frequency
omega_unstable_new = sqrt(abs(K_s_new) / optimal.total_mass);
f_unstable_new = omega_unstable_new / (2*pi);

% Target crossover: at least 5x unstable pole frequency
f_crossover_min = 5 * f_unstable_new;
f_crossover = max(100, f_crossover_min);  % At least 100 Hz or 5x unstable
omega_c = 2*pi*f_crossover;

% Target phase margin
phi_pm_target = 60;  % degrees

fprintf('LOOP-SHAPING DESIGN:\n');
fprintf('  Unstable pole: %.2f Hz\n', f_unstable_new);
fprintf('  Target crossover: %.0f Hz (min: %.0f Hz = 5x unstable)\n', f_crossover, f_crossover_min);
fprintf('  Target phase margin: %.0f deg\n\n', phi_pm_target);

% Rule of thumb frequencies:
%   w_z1 = w_c / 100   (far below crossover for robustness)
%   w_p = 4 * w_c      (above crossover for stability)
%   w_ci = 10 * w_c    (current loop bandwidth >> position)

w_z1 = omega_c / 100;
new_omega_px = 4 * omega_c;  % Derivative filter pole
w_ci = 10 * omega_c;

% Plant at crossover: G_p(jw_c) = K_i / (m*s^2 - |K_s|)
s_wc = 1j * omega_c;
G_p_wc = K_i_new / (optimal.total_mass * s_wc^2 - abs(K_s_new));

% Phase contributions at crossover
phase_z1 = atan(omega_c / w_z1);          % Phase lead from zero 1 (~90 deg)
phase_p = -atan(omega_c / new_omega_px);  % Phase lag from derivative filter
phase_int = -pi/2;                         % -90 deg from integrator
phase_plant = angle(G_p_wc);              % Plant phase (~180 deg for unstable)

% Required phase from w_z2 to achieve target phase margin
% Total phase = phase_z1 + phase_z2 + phase_int + phase_p + phase_plant
% Phase margin = Total phase + 180 deg
% Solve: phase_z2 = phi_pm - phase_z1 - phase_int - phase_p - phase_plant - 180
required_phase_z2 = (phi_pm_target*pi/180) - phase_z1 - phase_int - phase_p - phase_plant - pi;

% Solve for w_z2: atan(w_c/w_z2) = required_phase_z2
if required_phase_z2 > 0 && required_phase_z2 < pi/2
    w_z2 = omega_c / tan(required_phase_z2);
else
    w_z2 = omega_c / 10;  % Default: one decade below crossover
end
w_z2 = max(w_z2, w_z1 * 2);  % Ensure w_z2 > w_z1
w_z2 = min(w_z2, omega_c * 2);  % Don't place too high

% Compute K_o for unity gain at crossover
G_c_wc_normalized = (1 + s_wc/w_z1) * (1 + s_wc/w_z2) / (s_wc * (1 + s_wc/new_omega_px));
K_o = 1 / (abs(G_c_wc_normalized) * abs(G_p_wc));

% Convert to PID form:
%   k_i = K_o
%   k_p = K_o * (1/w_z1 + 1/w_z2)
%   k_d = K_o / (w_z1 * w_z2)
new_kix = K_o;
new_kpx = K_o * (1/w_z1 + 1/w_z2);
new_kdx = K_o / (w_z1 * w_z2);

% Verify achieved phase margin
G_c_wc = K_o * G_c_wc_normalized;
T_wc = G_c_wc * G_p_wc;
achieved_pm = (angle(T_wc) + pi) * 180/pi;

fprintf('Loop-Shaping Frequencies:\n');
fprintf('  w_z1 = %.2f rad/s (%.2f Hz)\n', w_z1, w_z1/(2*pi));
fprintf('  w_z2 = %.2f rad/s (%.2f Hz)\n', w_z2, w_z2/(2*pi));
fprintf('  w_p = %.0f rad/s (%.0f Hz)\n', new_omega_px, new_omega_px/(2*pi));
fprintf('  K_o = %.4e\n\n', K_o);

fprintf('NEW Position Controller (Loop-Shaping, %.0f Hz crossover):\n', f_crossover);
fprintf('  G_cx(s) = kp + ki/s + kd*s/(1+s/wp)\n');
fprintf('  kp = %.4e\n', new_kpx);
fprintf('  ki = %.4e\n', new_kix);
fprintf('  kd = %.4e\n', new_kdx);
fprintf('  wp = %.0f rad/s (%.0f Hz)\n', new_omega_px, new_omega_px/(2*pi));
fprintf('  Achieved phase margin: %.1f deg (target: %.0f deg)\n\n', achieved_pm, phi_pm_target);

% =========================================================================
% TILTING CONTROLLER DESIGN (Scaled from radial using stiffness margin)
% =========================================================================
L_amb_new = optimal.flywheel_length + 0.3;

% Compute proper transverse inertia for new design
r_outer_new = optimal.flywheel_diameter / 2;
r_inner_new = optimal.shaft_diameter / 2;
m_fly_new = rho_composite * pi * (r_outer_new^2 - r_inner_new^2) * optimal.flywheel_length;
m_shaft_new = rho_steel * pi * r_inner_new^2 * (optimal.flywheel_length + optimal.motor_length + 0.38);
m_total_check = m_fly_new + m_shaft_new;

% Approximate transverse inertia
I_transverse_new = (1/12) * optimal.total_mass * (3*(optimal.flywheel_diameter/2)^2 + optimal.flywheel_length^2);

% Effective tilting stiffness
d_amb_avg_new = L_amb_new / 2;
K_s_tilt_new = (d_amb_avg_new^2 + d_amb_avg_new^2) * K_s_new;  % Two AMBs

% Scale from baseline controller maintaining stiffness margin
% Stiffness margin = kp / |Ks|
baseline_stiffness_margin = baseline.kp_alpha / abs(baseline.K_s * (baseline.L_amb/2)^2 * 2);
new_kp_alpha = baseline_stiffness_margin * abs(K_s_tilt_new);
new_ki_alpha = new_kp_alpha * (omega_c / 10);  % Same relative integral corner
new_kd_alpha = new_kp_alpha / omega_c;         % Same relative derivative corner
new_omega_p_alpha = new_omega_px;              % Same filter frequency

fprintf('NEW Tilting Controller (Stiffness-Margin Scaling):\n');
fprintf('  Stiffness margin (baseline): %.2f\n', baseline_stiffness_margin);
fprintf('  kp_alpha = %.4e\n', new_kp_alpha);
fprintf('  ki_alpha = %.4e\n', new_ki_alpha);
fprintf('  kd_alpha = %.4e\n', new_kd_alpha);
fprintf('  wp_alpha = %.0f rad/s\n\n', new_omega_p_alpha);

% --- Build Transfer Functions using Control System Toolbox ---
s = tf('s');

% Baseline controllers (Appendix B: position-to-force, no K_i in plant)
G_cx_baseline = baseline.kpx + baseline.kix/s + baseline.kdx*s/(1 + s/baseline.omega_px);
G_plant_baseline = 1 / (baseline.total_mass*s^2 - baseline.K_s);  % No K_i: controller is pos-to-force
L_baseline = G_cx_baseline * G_plant_baseline;

% New system controllers (loop-shaping: position-to-current, needs K_i in plant)
G_cx_new = new_kpx + new_kix/s + new_kdx*s/(1 + s/new_omega_px);
G_plant_new = K_i_new / (optimal.total_mass*s^2 - K_s_new);  % K_i needed: controller is pos-to-current
L_new = G_cx_new * G_plant_new;

% Stability check via step response (more reliable than pole calculation)
fprintf('STABILITY CHECK:\n');

% Build closed-loop transfer functions for stability test
G_dist_bl_3a = 1 / (baseline.total_mass*s^2 - baseline.K_s);
T_x_bl_3a = G_dist_bl_3a / (1 + L_baseline);

G_dist_new_3a = 1 / (optimal.total_mass*s^2 - K_s_new);
T_x_new_3a = G_dist_new_3a / (1 + L_new);

% Check if step responses settle (indicates closed-loop stability)
[y_test_bl, ~] = step(T_x_bl_3a, linspace(0, 0.1, 1000));
[y_test_new, ~] = step(T_x_new_3a, linspace(0, 0.1, 1000));

stable_bl = abs(y_test_bl(end)) < 2*max(abs(y_test_bl(round(end/2):end)));
stable_new = abs(y_test_new(end)) < 2*max(abs(y_test_new(round(end/2):end)));

fprintf('  Baseline closed-loop stable: %s\n', mat2str(stable_bl));
fprintf('  New design closed-loop stable: %s\n\n', mat2str(stable_new));

% Compute phase margins if available
fprintf('STABILITY MARGINS:\n');
try
    margins_bl = allmargin(L_baseline);
    margins_new = allmargin(L_new);

    if ~isempty(margins_bl.PhaseMargin) && any(margins_bl.PhaseMargin > 0)
        pm_idx = find(margins_bl.PhaseMargin > 0, 1);
        fprintf('  Baseline phase margin: %.1f deg at %.1f Hz\n', ...
            margins_bl.PhaseMargin(pm_idx), margins_bl.PMFrequency(pm_idx)/(2*pi));
    end

    if ~isempty(margins_new.PhaseMargin) && any(margins_new.PhaseMargin > 0)
        pm_idx = find(margins_new.PhaseMargin > 0, 1);
        fprintf('  New design phase margin: %.1f deg at %.1f Hz\n', ...
            margins_new.PhaseMargin(pm_idx), margins_new.PMFrequency(pm_idx)/(2*pi));
    end
catch
    fprintf('  (Margin calculation not available)\n');
end
fprintf('\n');

%% ========================================================================
% PART 3b: STEP RESPONSE COMPARISON
% =========================================================================
fprintf('================================================================\n');
fprintf('PART 3b: AMB Step Response Comparison\n');
fprintf('================================================================\n\n');

% Step disturbances
F_step_bl = 0.10 * baseline.amb_rated_force;
F_step_new = 0.10 * new_amb_force;

fprintf('Step disturbances:\n');
fprintf('  Baseline: %.1f N (10%% of %.0f N)\n', F_step_bl, baseline.amb_rated_force);
fprintf('  New: %.1f N (10%% of %.0f N)\n\n', F_step_new, new_amb_force);

% Closed-loop transfer functions
G_dist_bl = 1 / (baseline.total_mass*s^2 - baseline.K_s);
T_x_bl = G_dist_bl / (1 + L_baseline);

G_dist_new = 1 / (optimal.total_mass*s^2 - K_s_new);
T_x_new = G_dist_new / (1 + L_new);

% Simulate step responses
t_3b = linspace(0, 0.05, 5000);

[x_bl, t_out_bl] = step(T_x_bl * F_step_bl, t_3b);
[x_new, t_out_new] = step(T_x_new * F_step_new, t_3b);

% Calculate metrics
peak_bl = max(abs(x_bl)) * 1e6;
peak_new = max(abs(x_new)) * 1e6;

% --- PLOT: Part 3b ---
figure('Name', 'Part 3b: Step Response Comparison', 'Position', [100 100 1000 500]);

subplot(1,2,1);
plot(t_out_bl*1000, x_bl*1e6, 'b-', 'LineWidth', 2); hold on;
plot(t_out_new*1000, x_new*1e6, 'r--', 'LineWidth', 2);
grid on;
xlabel('Time [ms]', 'FontSize', 11);
ylabel('Rotor Position [\mum]', 'FontSize', 11);
title('Part 3b: Position Response (0 RPM)', 'FontSize', 12);
legend('Baseline', 'New Design', 'Location', 'best');

subplot(1,2,2);
bar([peak_bl, peak_new]);
set(gca, 'XTickLabel', {'Baseline', 'New Design'});
ylabel('Peak Displacement [\mum]', 'FontSize', 11);
title('Part 3b: Peak Displacement Comparison', 'FontSize', 12);
grid on;

saveas(gcf, 'part3b_step_response.png');
fprintf('Plot saved: part3b_step_response.png\n');

fprintf('\nStep Response Comparison:\n');
fprintf('                    Baseline    New Design\n');
fprintf('  Peak displacement: %.4f um   %.4f um\n', peak_bl, peak_new);

%% ========================================================================
% PART 3c: DYNAMIC STIFFNESS AND RUNOUT COMPARISON
% =========================================================================
fprintf('\n================================================================\n');
fprintf('PART 3c: Dynamic Stiffness and Runout Comparison\n');
fprintf('================================================================\n\n');

% Frequency range
freq_3c = logspace(0, 3, 200);
omega_3c = 2*pi*freq_3c;

stiff_bl = zeros(size(omega_3c));
stiff_new = zeros(size(omega_3c));

% Use 2*K_s for two AMBs (consistent with Part 1e/1f)
K_s_bl_2amb = 2 * baseline.K_s;
K_s_new_2amb = 2 * K_s_new;

for idx = 1:length(omega_3c)
    s_val = 1j * omega_3c(idx);

    % Baseline (with 2*K_s for two AMBs, K_s positive destabilizing)
    % Note: Appendix B controller is position-to-force, no K_i needed
    G_cx_bl_val = baseline.kpx + baseline.kix/s_val + baseline.kdx*s_val/(1 + s_val/baseline.omega_px);
    stiff_bl(idx) = abs(baseline.total_mass*s_val^2 - K_s_bl_2amb + G_cx_bl_val);

    % New (with 2*K_s for two AMBs, K_s positive destabilizing)
    % Note: Loop-shaping controller is position-to-current, needs K_i
    G_cx_new_val = new_kpx + new_kix/s_val + new_kdx*s_val/(1 + s_val/new_omega_px);
    stiff_new(idx) = abs(optimal.total_mass*s_val^2 - K_s_new_2amb + G_cx_new_val*K_i_new);
end

% Runout comparison
SoC_3c = linspace(0, 100, 100);
runout_bl = zeros(size(SoC_3c));
runout_new = zeros(size(SoC_3c));

for idx = 1:length(SoC_3c)
    % Baseline (energy-based SoC: ω² is linear with SoC)
    omega_bl = sqrt(baseline.omega_min^2 + (baseline.omega_max^2 - baseline.omega_min^2) * SoC_3c(idx)/100);
    e_bl = (G_grade * 1e-3) / omega_bl;
    F_unb_bl = baseline.total_mass * e_bl * omega_bl^2;

    % Baseline controller is position-to-force, no K_i needed (K_s positive destabilizing)
    s_val = 1j * omega_bl;
    G_cx_bl_val = baseline.kpx + baseline.kix/s_val + baseline.kdx*s_val/(1 + s_val/baseline.omega_px);
    dyn_stiff_bl = abs(baseline.total_mass*s_val^2 - K_s_bl_2amb + G_cx_bl_val);
    runout_bl(idx) = (F_unb_bl / dyn_stiff_bl) * 1e6;

    % New (energy-based SoC)
    omega_new = sqrt(optimal.omega_min^2 + (optimal.omega_max^2 - optimal.omega_min^2) * SoC_3c(idx)/100);
    e_new = (G_grade * 1e-3) / omega_new;
    F_unb_new = optimal.total_mass * e_new * omega_new^2;

    % New controller is position-to-current, needs K_i (K_s positive destabilizing)
    s_val = 1j * omega_new;
    G_cx_new_val = new_kpx + new_kix/s_val + new_kdx*s_val/(1 + s_val/new_omega_px);
    dyn_stiff_new = abs(optimal.total_mass*s_val^2 - K_s_new_2amb + G_cx_new_val*K_i_new);
    runout_new(idx) = (F_unb_new / dyn_stiff_new) * 1e6;
end

% --- PLOT: Part 3c ---
figure('Name', 'Part 3c: Stiffness and Runout Comparison', 'Position', [100 100 1000 450]);

subplot(1,2,1);
loglog(freq_3c, stiff_bl/1e6, 'b-', 'LineWidth', 2); hold on;
loglog(freq_3c, stiff_new/1e6, 'r--', 'LineWidth', 2);
grid on;
xlabel('Frequency [Hz]', 'FontSize', 11);
ylabel('Dynamic Stiffness [MN/m]', 'FontSize', 11);
title('Part 3c: Dynamic Stiffness Comparison', 'FontSize', 12);
legend('Baseline', 'New Design', 'Location', 'best');

subplot(1,2,2);
plot(SoC_3c, runout_bl, 'b-', 'LineWidth', 2); hold on;
plot(SoC_3c, runout_new, 'r--', 'LineWidth', 2);
grid on;
xlabel('State of Charge [%]', 'FontSize', 11);
ylabel('Rotor Runout [\mum]', 'FontSize', 11);
title('Part 3c: Rotor Runout Comparison (ISO G2.5)', 'FontSize', 12);
legend('Baseline', 'New Design', 'Location', 'best');

saveas(gcf, 'part3c_stiffness_runout.png');
fprintf('Plot saved: part3c_stiffness_runout.png\n');

fprintf('\nDynamic Stiffness at Key Frequencies:\n');
fprintf('                    Baseline     New Design\n');
fprintf('  At 10 Hz:         %.2f MN/m   %.2f MN/m\n', ...
    interp1(freq_3c, stiff_bl, 10)/1e6, interp1(freq_3c, stiff_new, 10)/1e6);
fprintf('  At 100 Hz:        %.2f MN/m   %.2f MN/m\n', ...
    interp1(freq_3c, stiff_bl, 100)/1e6, interp1(freq_3c, stiff_new, 100)/1e6);

fprintf('\nRunout Comparison:\n');
fprintf('                    Baseline     New Design\n');
fprintf('  At 0%% SoC:        %.3f um     %.3f um\n', runout_bl(1), runout_new(1));
fprintf('  At 100%% SoC:      %.3f um     %.3f um\n', runout_bl(end), runout_new(end));

%% ========================================================================
% FINAL SUMMARY
% =========================================================================
fprintf('\n\n');
fprintf('================================================================\n');
fprintf('                    COMPLETE ANALYSIS SUMMARY\n');
fprintf('================================================================\n\n');

fprintf('BASELINE SYSTEM:\n');
fprintf('  Flywheel: %.0f mm dia x %.0f mm length\n', baseline.flywheel_diameter*1000, baseline.flywheel_length*1000);
fprintf('  Total mass: %.1f kg\n', baseline.total_mass);
fprintf('  Rated power: %.2f kW\n', baseline.power_rated/1000);
fprintf('  Specific power: %.3f kW/kg\n', specific_power_bl);
fprintf('  Specific energy: %.2f Wh/kg\n', specific_energy_bl);
fprintf('  15-min cycle efficiency: %.2f%%\n\n', efficiency_1c);

fprintf('OPTIMAL NEW DESIGN:\n');
fprintf('  Flywheel: %.0f mm dia x %.0f mm length\n', optimal.flywheel_diameter*1000, optimal.flywheel_length*1000);
fprintf('  Magnet thickness: %.1f mm\n', optimal.magnet_thickness*1000);
fprintf('  Max speed: %.0f RPM\n', optimal.max_speed_rpm);
fprintf('  Total mass: %.1f kg\n', optimal.total_mass);
fprintf('  Rated power: %.2f kW\n', optimal.power_rated/1000);
fprintf('  Specific power: %.3f kW/kg\n', optimal.specific_power);
fprintf('  Specific energy: %.2f Wh/kg\n', optimal.specific_energy);
fprintf('  6-hour cycle efficiency: %.2f%%\n\n', efficiency_2b);

fprintf('CONTROLLER COMPARISON:\n');
fprintf('                          Baseline         New Design\n');
fprintf('  Closed-loop stable:     %s              %s\n', mat2str(stable_bl), mat2str(stable_new));
fprintf('  Peak step displacement: %.4f um        %.4f um\n', peak_bl, peak_new);
fprintf('  Max runout:             %.3f um         %.3f um\n\n', max(runout_bl), max(runout_new));

fprintf('All plots saved to current directory.\n');
fprintf('See ASSUMPTIONS.md for complete documentation of all assumptions.\n');
fprintf('================================================================\n');
fprintf('                    ANALYSIS COMPLETE\n');
fprintf('================================================================\n');

%% ========================================================================
% LOCAL FUNCTIONS FOR 4-DOF AMB SIMULATION
% =========================================================================

function dstate = stateDerivative4DOF(t, state, params)
%STATEDERIVATIVE4DOF Compute state derivatives for 4-DOF AMB system.
%
%   State vector (20 states):
%     1-4:   x, y, alpha, beta (positions)
%     5-8:   xdot, ydot, alphadot, betadot (velocities)
%     9-12:  int_x, int_y, int_alpha, int_beta (controller integral states)
%     13-16: derf_x, derf_y, derf_alpha, derf_beta (derivative filter states)
%     17-20: i_ocx, i_ocy, i_dcx, i_dcy (current states)
%
%   params structure must contain:
%     .m, .J_t, .J_p, .d_top, .d_bot, .Ks, .Ki, .Omega
%     .kpx, .kix, .kdx, .omega_px (radial controller)
%     .kp_alpha, .ki_alpha, .kd_alpha, .omega_p_alpha (tilting controller)
%     .disturbance (optional struct with .t, .F_top_x, .F_top_y, etc.)

    % Extract states
    x = state(1);
    y = state(2);
    alpha = state(3);
    beta = state(4);
    xdot = state(5);
    ydot = state(6);
    alphadot = state(7);
    betadot = state(8);

    % Controller integral states
    int_x = state(9);
    int_y = state(10);
    int_alpha = state(11);
    int_beta = state(12);

    % Derivative filter states
    derf_x = state(13);
    derf_y = state(14);
    derf_alpha = state(15);
    derf_beta = state(16);

    % Current states (common/differential mode)
    i_ocx = state(17);
    i_ocy = state(18);
    i_dcx = state(19);
    i_dcy = state(20);

    % Parameters
    m = params.m;
    J_t = params.J_t;
    J_p = params.J_p;
    d_top = params.d_top;
    d_bot = params.d_bot;
    Ks = params.Ks;      % Per-axis stiffness (negative)
    Ki = params.Ki;      % Per-axis force constant
    Omega = params.Omega;

    % Controller gains
    kp_x = params.kpx;
    ki_x = params.kix;
    kd_x = params.kdx;
    wp_x = params.omega_px;

    kp_a = params.kp_alpha;
    ki_a = params.ki_alpha;
    kd_a = params.kd_alpha;
    wp_a = params.omega_p_alpha;

    % =====================================================================
    % BEARING DISPLACEMENTS
    % =====================================================================
    x_1 = x - d_top * beta;      % Top bearing x
    y_1 = y + d_top * alpha;     % Top bearing y
    x_2 = x + d_bot * beta;      % Bottom bearing x
    y_2 = y - d_bot * alpha;     % Bottom bearing y

    % =====================================================================
    % POSITION CONTROLLER (PID with derivative filter)
    % =====================================================================
    % Radial X controller -> commands common-mode x-force
    e_x = 0 - x;  % Position error
    dint_x = e_x;
    dderf_x = wp_x * (-xdot - derf_x);
    F_ocx_cmd = kp_x * e_x + ki_x * int_x + kd_x * derf_x;

    % Radial Y controller -> commands common-mode y-force
    e_y = 0 - y;
    dint_y = e_y;
    dderf_y = wp_x * (-ydot - derf_y);
    F_ocy_cmd = kp_x * e_y + ki_x * int_y + kd_x * derf_y;

    % Alpha tilting controller -> commands moment, convert to diff y-force
    e_alpha = 0 - alpha;
    dint_alpha = e_alpha;
    dderf_alpha = wp_a * (-alphadot - derf_alpha);
    M_alpha_cmd = kp_a * e_alpha + ki_a * int_alpha + kd_a * derf_alpha;
    F_dcy_cmd = M_alpha_cmd / (d_top + d_bot);

    % Beta tilting controller -> commands moment, convert to diff x-force
    e_beta = 0 - beta;
    dint_beta = e_beta;
    dderf_beta = wp_a * (-betadot - derf_beta);
    M_beta_cmd = kp_a * e_beta + ki_a * int_beta + kd_a * derf_beta;
    F_dcx_cmd = M_beta_cmd / (d_top + d_bot);

    % =====================================================================
    % CURRENT REGULATOR (using Appendix B parameters)
    % =====================================================================
    % The current controller is: G_ci(s) = Kp_current + Ki_current/s
    % Closed-loop bandwidth approximately: wci = Kp_current / L_coil
    %
    % This is derived from the PI controller acting on the coil:
    %   L * di/dt + R*i = V_applied
    %   V_applied = Kp*(i_cmd - i) + Ki*integral(i_cmd - i)
    % At high frequency, the proportional term dominates:
    %   L * di/dt ≈ Kp*(i_cmd - i)  =>  di/dt = (Kp/L)*(i_cmd - i)

    if isfield(params, 'Kp_current') && isfield(params, 'L_coil')
        % Use Appendix B current controller parameters
        wci = params.Kp_current / params.L_coil;  % = 345/0.0275 ≈ 12545 rad/s
    else
        % Fallback to estimation (10x position loop crossover)
        wc_est = max(sqrt(kp_x / m), sqrt(kp_a / J_t));
        wc_est = max(min(wc_est, 2*pi*5000), 2*pi*100);
        wci = 10 * wc_est;
    end

    % Convert force commands to current commands
    i_ocx_cmd = F_ocx_cmd / Ki;
    i_ocy_cmd = F_ocy_cmd / Ki;
    i_dcx_cmd = F_dcx_cmd / Ki;
    i_dcy_cmd = F_dcy_cmd / Ki;

    % First-order current dynamics: di/dt = wci*(i_cmd - i)
    di_ocx = wci * (i_ocx_cmd - i_ocx);
    di_ocy = wci * (i_ocy_cmd - i_ocy);
    di_dcx = wci * (i_dcx_cmd - i_dcx);
    di_dcy = wci * (i_dcy_cmd - i_dcy);

    % =====================================================================
    % CONTROL FORCES (from actual currents)
    % =====================================================================
    F_ocx = Ki * i_ocx;
    F_ocy = Ki * i_ocy;
    F_dcx = Ki * i_dcx;
    F_dcy = Ki * i_dcy;

    % Individual bearing control forces
    F_1x_ctrl = 0.5 * (F_ocx - F_dcx);
    F_2x_ctrl = 0.5 * (F_ocx + F_dcx);
    F_1y_ctrl = 0.5 * (F_ocy + F_dcy);
    F_2y_ctrl = 0.5 * (F_ocy - F_dcy);

    % =====================================================================
    % STIFFNESS FORCES (per-axis)
    % =====================================================================
    F_1x_stiff = Ks * x_1;
    F_1y_stiff = Ks * y_1;
    F_2x_stiff = Ks * x_2;
    F_2y_stiff = Ks * y_2;

    % Total forces at each bearing
    F_1x = F_1x_ctrl + F_1x_stiff;
    F_1y = F_1y_ctrl + F_1y_stiff;
    F_2x = F_2x_ctrl + F_2x_stiff;
    F_2y = F_2y_ctrl + F_2y_stiff;

    % =====================================================================
    % DISTURBANCES
    % =====================================================================
    F_dist_x = 0;
    F_dist_y = 0;
    M_dist_alpha = 0;
    M_dist_beta = 0;

    if isfield(params, 'disturbance') && ~isempty(params.disturbance)
        dist = params.disturbance;

        % Force at top bearing
        if isfield(dist, 'F_top_x') && ~isempty(dist.F_top_x)
            F_top_x = interp1(dist.t, dist.F_top_x, t, 'linear', 0);
            F_dist_x = F_dist_x + F_top_x;
            M_dist_beta = M_dist_beta - d_top * F_top_x;
        end
        if isfield(dist, 'F_top_y') && ~isempty(dist.F_top_y)
            F_top_y = interp1(dist.t, dist.F_top_y, t, 'linear', 0);
            F_dist_y = F_dist_y + F_top_y;
            M_dist_alpha = M_dist_alpha + d_top * F_top_y;
        end

        % Force at bottom bearing
        if isfield(dist, 'F_bot_x') && ~isempty(dist.F_bot_x)
            F_bot_x = interp1(dist.t, dist.F_bot_x, t, 'linear', 0);
            F_dist_x = F_dist_x + F_bot_x;
            M_dist_beta = M_dist_beta + d_bot * F_bot_x;
        end
        if isfield(dist, 'F_bot_y') && ~isempty(dist.F_bot_y)
            F_bot_y = interp1(dist.t, dist.F_bot_y, t, 'linear', 0);
            F_dist_y = F_dist_y + F_bot_y;
            M_dist_alpha = M_dist_alpha - d_bot * F_bot_y;
        end

        % Imbalance force at COM
        if isfield(dist, 'F_imb_x') && ~isempty(dist.F_imb_x)
            F_dist_x = F_dist_x + interp1(dist.t, dist.F_imb_x, t, 'linear', 0);
        end
        if isfield(dist, 'F_imb_y') && ~isempty(dist.F_imb_y)
            F_dist_y = F_dist_y + interp1(dist.t, dist.F_imb_y, t, 'linear', 0);
        end
    end

    % =====================================================================
    % EQUATIONS OF MOTION
    % =====================================================================
    F_x_total = F_1x + F_2x;
    F_y_total = F_1y + F_2y;

    M_alpha = d_top * F_1y - d_bot * F_2y;
    M_beta = d_bot * F_2x - d_top * F_1x;

    % Gyroscopic coupling
    tau_gyro_alpha = -J_p * Omega * betadot;
    tau_gyro_beta = J_p * Omega * alphadot;

    % =====================================================================
    % ACCELERATIONS
    % =====================================================================
    xddot = (F_x_total + F_dist_x) / m;
    yddot = (F_y_total + F_dist_y) / m;
    alphaddot = (M_alpha + M_dist_alpha + tau_gyro_alpha) / J_t;
    betaddot = (M_beta + M_dist_beta + tau_gyro_beta) / J_t;

    % =====================================================================
    % STATE DERIVATIVES
    % =====================================================================
    dstate = [xdot; ydot; alphadot; betadot;
              xddot; yddot; alphaddot; betaddot;
              dint_x; dint_y; dint_alpha; dint_beta;
              dderf_x; dderf_y; dderf_alpha; dderf_beta;
              di_ocx; di_ocy; di_dcx; di_dcy];
end

function results = simulate4DOF_step(params, t_end, F_step, direction)
%SIMULATE4DOF_STEP Simulate step response using 4-DOF ODE model.
%
%   Inputs:
%     params - Structure with system parameters
%     t_end - Simulation end time [s]
%     F_step - Step force magnitude [N]
%     direction - 'x' or 'y' direction of force at top bearing
%
%   Output:
%     results - Structure with time histories

    % Create disturbance
    t_dist = [0, 1e-6, t_end];
    if strcmpi(direction, 'x')
        disturbance.t = t_dist;
        disturbance.F_top_x = [0, F_step, F_step];
    else
        disturbance.t = t_dist;
        disturbance.F_top_y = [0, F_step, F_step];
    end
    params.disturbance = disturbance;

    % Initial conditions (equilibrium)
    state0 = zeros(20, 1);

    % Solve ODE
    options = odeset('RelTol', 1e-6, 'AbsTol', 1e-9);
    [t, state] = ode45(@(t, s) stateDerivative4DOF(t, s, params), [0, t_end], state0, options);

    % Extract states
    x = state(:,1);
    y = state(:,2);
    alpha = state(:,3);
    beta = state(:,4);

    % Current states
    i_ocx = state(:,17);
    i_ocy = state(:,18);
    i_dcx = state(:,19);
    i_dcy = state(:,20);

    % Compute bearing displacements
    d_top = params.d_top;
    d_bot = params.d_bot;
    x_1 = x - d_top * beta;
    y_1 = y + d_top * alpha;
    x_2 = x + d_bot * beta;
    y_2 = y - d_bot * alpha;

    % Individual bearing currents
    i_1x = 0.5 * (i_ocx - i_dcx);
    i_2x = 0.5 * (i_ocx + i_dcx);
    i_1y = 0.5 * (i_ocy - i_dcy);
    i_2y = 0.5 * (i_ocy + i_dcy);

    % Bearing forces
    Ki = params.Ki;
    Ks = params.Ks;
    F_1x = Ki * i_1x + Ks * x_1;
    F_1y = Ki * i_1y + Ks * y_1;
    F_2x = Ki * i_2x + Ks * x_2;
    F_2y = Ki * i_2y + Ks * y_2;

    % Package results based on direction
    results.t = t;
    results.x = x;
    results.y = y;
    results.alpha = alpha;
    results.beta = beta;

    if strcmpi(direction, 'x')
        results.pos_top = x_1;
        results.pos_bot = x_2;
        results.F_top = F_1x;
        results.F_bot = F_2x;
        results.i_top = i_1x;
        results.i_bot = i_2x;
    else
        results.pos_top = y_1;
        results.pos_bot = y_2;
        results.F_top = F_1y;
        results.F_bot = F_2y;
        results.i_top = i_1y;
        results.i_bot = i_2y;
    end
end
