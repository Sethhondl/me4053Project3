%% TEAM 16 - FLYWHEEL ENERGY STORAGE SYSTEM ANALYSIS
% =========================================================================
% Course: Mechanical Engineering Modeling
% Project: Flywheel Energy Storage System Analysis for eXtreme Storage Inc.
%
% This script performs comprehensive analysis of a flywheel energy storage
% system including baseline characterization, design optimization, and
% AMB controller design.
%
% DELIVERABLES:
%   Deliverable 1: Baseline System Characterization (Parts 1a-1f)
%   Deliverable 2: New Design Development (Parts 2a-2b)
%   Deliverable 3: AMB Controller Design (Parts 3a-3c)
%
% REQUIRED P-CODE FILES (included in this directory):
%   - magneticShear.p      : Magnetic shear stress calculation
%   - rotorLosses.p        : Rotor electromagnetic losses
%   - statorLosses.p       : Stator electromagnetic losses
%   - ambParameters.p      : Active magnetic bearing parameters
%   - baselineStorageCycle.p : 15-minute baseline storage cycle
%   - team_16_cycle.p      : Team 16's 6-hour storage cycle
%   - genericStorageCycle.p : Generic storage cycle
%   - elecMachineParams.p  : Electrical machine parameters
%
% Author: Team 16
% Date: 2025-11-25
% =========================================================================

clear; close all; clc;

%% ========================================================================
% INITIALIZATION
% =========================================================================
% Add current directory to path (contains all .p files)
% This ensures the script is self-contained and works from this directory
addpath(pwd);

fprintf('================================================================\n');
fprintf('TEAM 16 - FLYWHEEL ENERGY STORAGE SYSTEM ANALYSIS\n');
fprintf('================================================================\n');
fprintf('Date: %s\n\n', datestr(now));

%% ########################################################################
%                         DELIVERABLE 1
%                   BASELINE SYSTEM CHARACTERIZATION
% #########################################################################

fprintf('\n');
fprintf('================================================================\n');
fprintf('              DELIVERABLE 1: BASELINE ANALYSIS\n');
fprintf('================================================================\n\n');

%% ========================================================================
% DELIVERABLE 1 - SYSTEM PARAMETERS
% =========================================================================
% Load baseline system specifications from Appendix B - Table A.1
% These are fixed parameters for the existing flywheel design.
%
% ASSUMPTIONS:
%   1. Shaft length estimated at 1.5 m (includes motor, AMBs, clearances)
%   2. AMB rotor components add ~10% to shaft mass
%   3. Rated current is THERMALLY LIMITED to I_rated_pu = 0.70 (NOT 0.8!)
%      - At 100% SoC with I=0.70 pu, rotor losses ~390 W -> T ~100C
%      - Higher current would exceed the 100C temperature limit
%   4. Housing temperature maintained at 30C (from Appendix B)
% =========================================================================

fprintf('Loading baseline system parameters...\n\n');

% Geometric parameters [m] (from Appendix B - Table A.1)
baseline.flywheel_length = 1.000;      % Flywheel axial length [m]
baseline.flywheel_diameter = 0.430;    % Flywheel diameter [m]
baseline.motor_length = 0.250;         % Motor axial length [m]
baseline.shaft_diameter = 0.084;       % Shaft and PM diameter [m]
baseline.magnet_thickness = 0.006;     % Magnet thickness [m]

% Operational parameters
baseline.max_speed_rpm = 40000;        % Maximum rotational speed [r/min]
baseline.amb_rated_force = 5780;       % AMB rated force [N]

% Material properties (from Table 1)
rho_composite = 1600;                  % Composite density [kg/m^3]
rho_steel = 7850;                      % Steel density [kg/m^3]
rho_magnet = 7850;                     % Permanent magnet density [kg/m^3]

% Safety limits (from Table 1)
max_steel_tip_speed = 175;             % Max steel tip speed [m/s]
max_pm_tip_speed = 175;                % Max PM tip speed [m/s]
max_composite_tip_speed = 900;         % Max composite tip speed [m/s]
max_temp = 100;                        % Max safe temperature [C]

% State of charge definition
% 0% SoC = 50% of max speed, 100% SoC = 100% of max speed
baseline.min_speed_rpm = baseline.max_speed_rpm / 2;  % 20,000 r/min

% Convert to rad/s
baseline.omega_max = baseline.max_speed_rpm * 2*pi / 60;  % [rad/s]
baseline.omega_min = baseline.min_speed_rpm * 2*pi / 60;  % [rad/s]

% Thermal parameters (from Table 1)
epsilon_rotor = 0.4;                   % Rotor emissivity
epsilon_housing = 0.9;                 % Housing emissivity
sigma = 5.67e-8;                       % Stefan-Boltzmann constant [W/(m^2*K^4)]
T_housing_C = 30;                      % Housing temperature [C] (from Appendix B)
T_housing_K = T_housing_C + 273;       % Housing temperature [K]

% Controller parameters from Appendix B
baseline.Kp_current = 345;             % Current controller proportional
baseline.Ki_current = 2149;            % Current controller integral
baseline.kpx = 1.2639e8;               % Position controller X proportional
baseline.kix = 1.16868e9;              % Position controller X integral
baseline.kdx = 252790;                 % Position controller X derivative
baseline.omega_px = 3770;              % Derivative filter cutoff [rad/s]
baseline.kp_alpha = 7.6992e7;          % Tilting controller proportional
baseline.ki_alpha = 1.18953e9;         % Tilting controller integral
baseline.kd_alpha = 80294;             % Tilting controller derivative
baseline.omega_p_alpha = 6283;         % Tilting derivative filter [rad/s]

fprintf('Baseline System Specifications:\n');
fprintf('  Flywheel: %.0f mm diameter x %.0f mm length\n', ...
    baseline.flywheel_diameter*1000, baseline.flywheel_length*1000);
fprintf('  Shaft: %.0f mm diameter\n', baseline.shaft_diameter*1000);
fprintf('  Motor: %.0f mm length, %.1f mm magnet thickness\n', ...
    baseline.motor_length*1000, baseline.magnet_thickness*1000);
fprintf('  Speed range: %.0f - %.0f RPM\n\n', baseline.min_speed_rpm, baseline.max_speed_rpm);

%% ========================================================================
% DELIVERABLE 1 - MASS AND INERTIA CALCULATIONS
% =========================================================================
% Calculate rotating group mass and moment of inertia.
%
% GEOMETRY:
%   - Flywheel: hollow composite cylinder (outer = flywheel_dia, inner = shaft_dia)
%   - Shaft: solid steel cylinder (length = 1.5 m estimated)
%   - Magnets: hollow cylinder on motor section
%   - AMB rotors: estimated as 10% of shaft mass
%
% MOMENT OF INERTIA:
%   - Hollow cylinder: I = 0.5 * m * (r_outer^2 + r_inner^2)
%   - Solid cylinder: I = 0.5 * m * r^2
% =========================================================================

fprintf('Calculating rotating group mass and inertia...\n');

% Flywheel mass (hollow composite cylinder)
r_outer_bl = baseline.flywheel_diameter / 2;
r_inner_bl = baseline.shaft_diameter / 2;
V_flywheel_bl = pi * (r_outer_bl^2 - r_inner_bl^2) * baseline.flywheel_length;
m_flywheel_bl = rho_composite * V_flywheel_bl;

% Shaft mass (solid steel cylinder)
% ASSUMPTION: Total shaft length = 1.5 m (flywheel + motor + AMBs + clearances)
shaft_length_bl = 1.5;  % [m]
V_shaft_bl = pi * r_inner_bl^2 * shaft_length_bl;
m_shaft_bl = rho_steel * V_shaft_bl;

% Permanent magnet mass (hollow cylinder on motor section)
r_mag_outer_bl = r_inner_bl + baseline.magnet_thickness;
V_magnet_bl = pi * (r_mag_outer_bl^2 - r_inner_bl^2) * baseline.motor_length;
m_magnet_bl = rho_magnet * V_magnet_bl;

% AMB rotor components (ASSUMPTION: 10% of shaft mass)
m_amb_rotors_bl = 0.10 * m_shaft_bl;

% Total rotating group mass
baseline.total_mass = m_flywheel_bl + m_shaft_bl + m_magnet_bl + m_amb_rotors_bl;

% Moment of inertia calculations
I_flywheel_bl = 0.5 * m_flywheel_bl * (r_outer_bl^2 + r_inner_bl^2);  % Hollow cylinder
I_shaft_bl = 0.5 * m_shaft_bl * r_inner_bl^2;                         % Solid cylinder
I_magnet_bl = 0.5 * m_magnet_bl * (r_mag_outer_bl^2 + r_inner_bl^2); % Hollow cylinder
baseline.I_total = I_flywheel_bl + I_shaft_bl + I_magnet_bl;

fprintf('  Flywheel mass: %.2f kg\n', m_flywheel_bl);
fprintf('  Shaft mass: %.2f kg\n', m_shaft_bl);
fprintf('  Magnet mass: %.2f kg\n', m_magnet_bl);
fprintf('  AMB rotors: %.2f kg\n', m_amb_rotors_bl);
fprintf('  Total rotating mass: %.2f kg\n', baseline.total_mass);
fprintf('  Moment of inertia: %.4f kg*m^2\n\n', baseline.I_total);

%% ========================================================================
% DELIVERABLE 1a: LOSSES AND TEMPERATURE VS STATE OF CHARGE
% =========================================================================
% Calculate motor losses (rotor and stator) and rotor temperature as a
% function of state of charge while operating at rated power.
%
% PROCESS:
%   1. Sweep SoC from 0% to 100%
%   2. Calculate speed: omega = omega_min + (omega_max - omega_min) * SoC/100
%   3. Calculate current needed for rated power at each speed
%   4. Get losses from EE functions (rotorLosses, statorLosses)
%   5. Calculate temperature using radiation heat transfer
%
% ASSUMPTIONS:
%   1. Rated current is THERMALLY LIMITED: I_rated_pu = 0.70 (NOT 0.8!)
%      - With low rotor emissivity (0.4), heat dissipation is limited
%      - Max rotor loss for T <= 100C is ~390 W
%      - At I=0.70 pu and max speed, rotor losses ~390 W -> T ~100C
%   2. Current scales inversely with speed to maintain constant power
%   3. Heat transfer via radiation only (vacuum environment)
%   4. Only rotor losses heat the rotor (stator is outside vacuum)
%   5. Two-surface enclosure radiation model
%   6. Housing radial clearance = 20 mm from flywheel
%
% THERMAL MODEL:
%   Q = sigma * A * (T_rotor^4 - T_housing^4) / F_rad
%   F_rad = 1/eps_rotor + (A_rotor/A_housing)*(1/eps_housing - 1) ~ 2.6
% =========================================================================

fprintf('================================================================\n');
fprintf('PART 1a: Losses and Temperature vs State of Charge\n');
fprintf('================================================================\n\n');

% Define SoC range
SoC_range = linspace(0, 100, 50);

% Calculate rated power from motor parameters
% CRITICAL: Rated current is THERMALLY LIMITED to 0.70 pu (NOT 0.8!)
% At I=0.70 pu and max speed, rotor losses ~390 W which gives T ~100C
I_rated_pu = 0.70;
shear_rated = magneticShear(baseline.magnet_thickness, I_rated_pu);
motor_area = pi * baseline.shaft_diameter * baseline.motor_length;
motor_radius = baseline.shaft_diameter / 2;
torque_rated = shear_rated * motor_area * motor_radius;
baseline.power_rated = torque_rated * baseline.omega_max;

fprintf('Motor Parameters:\n');
fprintf('  Rated current: %.2f pu (THERMALLY LIMITED - not 0.8!)\n', I_rated_pu);
fprintf('  Magnetic shear: %.2f Pa\n', shear_rated);
fprintf('  Rated torque: %.2f Nm\n', torque_rated);
fprintf('  Rated power: %.2f kW\n\n', baseline.power_rated/1000);

% Initialize arrays
omega_array_1a = zeros(size(SoC_range));
rotor_loss_array = zeros(size(SoC_range));
stator_loss_array = zeros(size(SoC_range));
total_loss_array = zeros(size(SoC_range));
temperature_array = zeros(size(SoC_range));
current_array = zeros(size(SoC_range));

% Rotor surface area for thermal calculation
A_rotor = 2*pi*r_outer_bl*baseline.flywheel_length + 2*pi*r_outer_bl^2;

% Housing surface area (with 20mm radial clearance)
housing_clearance = 0.020;  % [m]
housing_inner_dia = baseline.flywheel_diameter + 2*housing_clearance;
A_housing = 2*pi*(housing_inner_dia/2)*baseline.flywheel_length + 2*pi*(housing_inner_dia/2)^2;

% Two-surface radiation factor
rad_factor = 1/epsilon_rotor + (A_rotor/A_housing)*(1/epsilon_housing - 1);

fprintf('Calculating losses across SoC range...\n');

for i = 1:length(SoC_range)
    % Calculate speed for this SoC
    omega = baseline.omega_min + (baseline.omega_max - baseline.omega_min) * SoC_range(i)/100;
    omega_array_1a(i) = omega;
    omega_rpm = omega * 60 / (2*pi);

    % Current needed for rated power at this speed
    % Power = shear * A * r * omega, shear proportional to current
    % To maintain constant power: I scales as omega_max/omega
    I_pu = I_rated_pu * (baseline.omega_max / omega);
    I_pu = min(I_pu, 1.0);  % Limit to maximum
    current_array(i) = I_pu;

    % Get losses from EE functions
    % Note: rotorLosses and statorLosses expect RPM, not rad/s
    P_rotor = rotorLosses(baseline.magnet_thickness, baseline.shaft_diameter, ...
        baseline.motor_length, I_pu, omega_rpm);
    P_stator = statorLosses(baseline.magnet_thickness, baseline.shaft_diameter, ...
        baseline.motor_length, I_pu, omega_rpm);

    rotor_loss_array(i) = P_rotor;
    stator_loss_array(i) = P_stator;
    total_loss_array(i) = P_rotor + P_stator;

    % Temperature calculation using two-surface radiation
    % Only ROTOR losses heat the rotor (stator is outside vacuum)
    % Q = sigma * A * (T_r^4 - T_h^4) / rad_factor
    % Solving for T_r: T_r = (T_h^4 + Q*rad_factor/(sigma*A))^0.25
    T_rotor_K = (T_housing_K^4 + P_rotor*rad_factor/(sigma*A_rotor))^0.25;
    temperature_array(i) = T_rotor_K - 273;  % Convert to Celsius
end

% Print results
fprintf('\nResults at Key SoC Points:\n');
fprintf('  SoC    Speed     Current   Rotor Loss  Stator Loss  Total Loss  Temp\n');
fprintf('  [%%]    [RPM]     [pu]      [kW]        [kW]         [kW]        [C]\n');
fprintf('  --------------------------------------------------------------------\n');
idx_0 = 1;
idx_50 = round(length(SoC_range)/2);
idx_100 = length(SoC_range);
for idx = [idx_0, idx_50, idx_100]
    fprintf('  %3.0f    %5.0f     %.2f      %.2f        %.2f         %.2f        %.1f\n', ...
        SoC_range(idx), omega_array_1a(idx)*60/(2*pi), current_array(idx), ...
        rotor_loss_array(idx)/1000, stator_loss_array(idx)/1000, ...
        total_loss_array(idx)/1000, temperature_array(idx));
end

% === PLOT: Part 1a - Losses and Temperature ===
figure('Name', 'Part 1a: Losses and Temperature', 'Position', [100 100 1000 450]);

subplot(1,2,1);
plot(SoC_range, rotor_loss_array/1000, 'r-', 'LineWidth', 2); hold on;
plot(SoC_range, stator_loss_array/1000, 'b-', 'LineWidth', 2);
plot(SoC_range, total_loss_array/1000, 'k-', 'LineWidth', 2.5);
grid on;
xlabel('State of Charge [%]', 'FontSize', 12);
ylabel('Power Loss [kW]', 'FontSize', 12);
title('Part 1a: Losses at Rated Power vs SoC', 'FontSize', 14);
legend('Rotor Losses', 'Stator Losses', 'Total Losses', 'Location', 'best');

subplot(1,2,2);
plot(SoC_range, temperature_array, 'k-', 'LineWidth', 2.5); hold on;
yline(max_temp, 'r--', 'LineWidth', 2, 'Label', 'Max Safe (100C)');
grid on;
xlabel('State of Charge [%]', 'FontSize', 12);
ylabel('Rotor Temperature [C]', 'FontSize', 12);
title('Part 1a: Rotor Temperature vs SoC', 'FontSize', 14);

saveas(gcf, 'part1a_losses_temperature.png');
fprintf('\nPlot saved: part1a_losses_temperature.png\n');

fprintf('\nPart 1a Summary:\n');
fprintf('  Max total losses: %.2f kW at %.0f%% SoC\n', max(total_loss_array)/1000, ...
    SoC_range(total_loss_array == max(total_loss_array)));
fprintf('  Max temperature: %.1f C (limit: %.0f C)\n', max(temperature_array), max_temp);

%% ========================================================================
% DELIVERABLE 1b: SPECIFIC POWER AND SPECIFIC ENERGY
% =========================================================================
% Calculate the specific power [kW/kg] and specific energy [Wh/kg] of the
% baseline flywheel system.
%
% DEFINITIONS:
%   - Specific Power = Rated Power / Total Mass
%   - Specific Energy = Usable Energy / Total Mass
%   - Usable Energy = E(100% SoC) - E(0% SoC) = 0.5*I*(omega_max^2 - omega_min^2)
%
% Note: Energy is stored as kinetic energy: E = 0.5 * I * omega^2
% =========================================================================

fprintf('\n================================================================\n');
fprintf('PART 1b: Specific Power and Specific Energy\n');
fprintf('================================================================\n\n');

% Specific power [kW/kg]
specific_power_bl = (baseline.power_rated / 1000) / baseline.total_mass;

% Energy calculations
E_max = 0.5 * baseline.I_total * baseline.omega_max^2;  % [J] at 100% SoC
E_min = 0.5 * baseline.I_total * baseline.omega_min^2;  % [J] at 0% SoC
E_stored = E_max - E_min;                                % [J] usable energy
E_stored_Wh = E_stored / 3600;                          % [Wh]
E_stored_kWh = E_stored_Wh / 1000;                      % [kWh]

% Specific energy [Wh/kg]
specific_energy_bl = E_stored_Wh / baseline.total_mass;

fprintf('Energy Storage:\n');
fprintf('  Energy at 100%% SoC: %.2f kWh\n', E_max/3.6e6);
fprintf('  Energy at 0%% SoC: %.2f kWh\n', E_min/3.6e6);
fprintf('  Usable energy: %.2f kWh\n', E_stored_kWh);
fprintf('\nSpecific Performance:\n');
fprintf('  Specific Power: %.3f kW/kg\n', specific_power_bl);
fprintf('  Specific Energy: %.2f Wh/kg\n', specific_energy_bl);

%% ========================================================================
% DELIVERABLE 1c: STORAGE CYCLE EFFICIENCY
% =========================================================================
% Simulate the 15-minute baseline storage cycle and calculate round-trip
% efficiency.
%
% PROCESS:
%   1. Start at 50% SoC
%   2. Follow power demand from baselineStorageCycle(t)
%   3. Track energy flows: charging (grid -> flywheel), discharging (flywheel -> grid)
%   4. Account for losses at each timestep
%   5. Calculate efficiency = E_out / E_in
%
% ASSUMPTIONS:
%   1. Current scales with power demand relative to rated power
%   2. Losses calculated at each operating point
%   3. Speed updated based on energy balance
%
% EFFICIENCY CALCULATION:
%   eta = E_discharged / (E_charged + E_recovery)
%   where E_recovery restores the flywheel to initial SoC
% =========================================================================

fprintf('\n================================================================\n');
fprintf('PART 1c: Storage Cycle Efficiency (15-minute baseline)\n');
fprintf('================================================================\n\n');

% Simulation parameters
dt = 1.0;           % Time step [s]
t_cycle = 0:dt:900; % 15-minute cycle [s]
SoC_initial = 50;   % Start at 50% SoC

% Initial state
omega_current = baseline.omega_min + (baseline.omega_max - baseline.omega_min) * SoC_initial/100;
E_current = 0.5 * baseline.I_total * omega_current^2;

% Storage arrays
omega_cycle = zeros(size(t_cycle));
SoC_cycle = zeros(size(t_cycle));
power_grid = zeros(size(t_cycle));
power_losses = zeros(size(t_cycle));

% Energy tracking
E_in = 0;   % Energy from grid (charging)
E_out = 0;  % Energy to grid (discharging)

fprintf('Simulating 15-minute storage cycle...\n');

for i = 1:length(t_cycle)
    % Get grid power demand (positive = discharge to grid)
    P_grid = baselineStorageCycle(t_cycle(i));
    power_grid(i) = P_grid;

    % Current speed and SoC
    omega_rpm = omega_current * 60 / (2*pi);
    SoC_current = 100 * (omega_current - baseline.omega_min) / (baseline.omega_max - baseline.omega_min);

    % Calculate current for this power level
    I_pu = I_rated_pu * abs(P_grid) / baseline.power_rated;
    I_pu = min(I_pu, 1.0);

    % Get losses
    P_rotor = rotorLosses(baseline.magnet_thickness, baseline.shaft_diameter, ...
        baseline.motor_length, I_pu, omega_rpm);
    P_stator = statorLosses(baseline.magnet_thickness, baseline.shaft_diameter, ...
        baseline.motor_length, I_pu, omega_rpm);
    P_loss = P_rotor + P_stator;
    power_losses(i) = P_loss;

    % Energy balance: dE/dt = -P_grid - P_loss (negative because P_grid>0 = discharge)
    P_net = P_grid + P_loss;
    dE = -P_net * dt;
    E_current = E_current + dE;

    % Update speed from energy
    omega_current = sqrt(max(0, 2 * E_current / baseline.I_total));
    omega_current = max(baseline.omega_min, min(baseline.omega_max, omega_current));

    % Store values
    omega_cycle(i) = omega_current;
    SoC_cycle(i) = 100 * (omega_current - baseline.omega_min) / (baseline.omega_max - baseline.omega_min);

    % Track energy flows
    if P_grid > 0
        E_out = E_out + P_grid * dt;
    else
        E_in = E_in + abs(P_grid) * dt;
    end
end

% Calculate efficiency
E_loss_total = sum(power_losses) * dt;
SoC_final = SoC_cycle(end);
E_recovery = max(0, 0.5 * baseline.I_total * ((baseline.omega_min + (baseline.omega_max - baseline.omega_min)*SoC_initial/100)^2 - omega_cycle(end)^2));

efficiency_1c = (E_out / (E_in + E_recovery)) * 100;

% === PLOT: Part 1c - Storage Cycle ===
figure('Name', 'Part 1c: Storage Cycle', 'Position', [100 100 1000 700]);

subplot(3,1,1);
plot(t_cycle/60, power_grid/1000, 'b-', 'LineWidth', 2);
grid on;
xlabel('Time [min]', 'FontSize', 11);
ylabel('Grid Power [kW]', 'FontSize', 11);
title('Part 1c: Baseline Storage Cycle - Power Demand', 'FontSize', 12);

subplot(3,1,2);
plot(t_cycle/60, SoC_cycle, 'r-', 'LineWidth', 2);
grid on;
xlabel('Time [min]', 'FontSize', 11);
ylabel('State of Charge [%]', 'FontSize', 11);
title('Part 1c: Flywheel SoC During Cycle', 'FontSize', 12);

subplot(3,1,3);
plot(t_cycle/60, power_losses/1000, 'k-', 'LineWidth', 2);
grid on;
xlabel('Time [min]', 'FontSize', 11);
ylabel('Total Losses [kW]', 'FontSize', 11);
title('Part 1c: Motor Losses During Cycle', 'FontSize', 12);

saveas(gcf, 'part1c_storage_cycle.png');
fprintf('Plot saved: part1c_storage_cycle.png\n');

fprintf('\nCycle Results:\n');
fprintf('  Energy discharged to grid: %.2f kWh\n', E_out/3.6e6);
fprintf('  Energy charged from grid: %.2f kWh\n', E_in/3.6e6);
fprintf('  Total losses: %.2f kWh\n', E_loss_total/3.6e6);
fprintf('  Final SoC: %.1f%% (started at %.0f%%)\n', SoC_final, SoC_initial);
fprintf('  Cycle Efficiency: %.2f%%\n', efficiency_1c);

%% ========================================================================
% DELIVERABLE 1d: AMB STEP RESPONSE
% =========================================================================
% Analyze the step response of the active magnetic bearing (AMB) system
% to a 10% rated force disturbance at 0 RPM.
%
% SYSTEM MODEL:
%   - Plant: G_p(s) = K_i / (m*s^2 - K_s)
%   - K_s < 0 (negative/destabilizing stiffness)
%   - Position controller: G_cx(s) = kp + ki/s + kd*s/(1+s/wp)
%
% TRANSFER FUNCTION APPROACH:
%   Closed-loop from disturbance to position:
%   X(s)/F_dist(s) = G_dist / (1 + L)
%   where L = G_cx * G_ci * G_plant
%
% ASSUMPTIONS:
%   1. Current loop much faster than position loop (G_ci ≈ 1)
%   2. Rigid rotor (no bending modes)
%   3. Controller parameters from Appendix B
% =========================================================================

fprintf('\n================================================================\n');
fprintf('PART 1d: AMB Step Response Analysis\n');
fprintf('================================================================\n\n');

% Get AMB parameters
amb_bl = ambParameters(baseline.shaft_diameter, baseline.amb_rated_force);

K_s_bl = -amb_bl.stiffnessConstant;  % Negative stiffness (destabilizing) [N/m]
K_i_bl = amb_bl.forceConstant;        % Current-to-force constant [N/A]

fprintf('AMB Parameters:\n');
fprintf('  Position stiffness (K_s): %.2e N/m (negative/destabilizing)\n', K_s_bl);
fprintf('  Force constant (K_i): %.2f N/A\n', K_i_bl);
fprintf('  Coil inductance: %.4f H\n', amb_bl.coilInductance);
fprintf('  Coil resistance: %.3f Ohms\n\n', amb_bl.coilResistance);

% Step disturbance: 10% of rated force
F_step = 0.10 * baseline.amb_rated_force;
fprintf('Step disturbance: %.1f N (10%% of %.0f N rated)\n\n', F_step, baseline.amb_rated_force);

% Build transfer function model
s = tf('s');

% Position controller: G_cx(s) = kp + ki/s + kd*s/(1+s/wp)
G_cx = baseline.kpx + baseline.kix/s + s*baseline.kdx/(1 + s/baseline.omega_px);

% Plant: X(s)/F(s) = 1/(m*s^2 - K_s)
% Note: K_s is already negative
G_plant = K_i_bl / (baseline.total_mass*s^2 + K_s_bl);

% Open-loop
L = G_cx * G_plant;

% Closed-loop from disturbance to position
G_dist = 1 / (baseline.total_mass*s^2 + K_s_bl);
T_dist = G_dist / (1 + L);

% Closed-loop from disturbance to control force
T_force = -L * G_dist / (1 + L);

% Simulate step response
t_amb = linspace(0, 0.05, 5000);  % 50 ms

[x_response, t_out] = step(T_dist * F_step, t_amb);
[f_ctrl, ~] = step(T_force * F_step * K_i_bl, t_amb);
i_response = f_ctrl / K_i_bl;

% === PLOT: Part 1d - AMB Step Response ===
figure('Name', 'Part 1d: AMB Step Response', 'Position', [100 100 1000 700]);

subplot(3,1,1);
plot(t_out*1000, i_response, 'b-', 'LineWidth', 2);
grid on;
xlabel('Time [ms]', 'FontSize', 11);
ylabel('Current [A]', 'FontSize', 11);
title('Part 1d: AMB Current Response (0 RPM)', 'FontSize', 12);

subplot(3,1,2);
plot(t_out*1000, f_ctrl, 'r-', 'LineWidth', 2);
hold on;
yline(-F_step, 'k--', 'LineWidth', 1.5);
grid on;
xlabel('Time [ms]', 'FontSize', 11);
ylabel('AMB Force [N]', 'FontSize', 11);
title('Part 1d: AMB Force Response (0 RPM)', 'FontSize', 12);
legend('AMB Force', 'Target (-F_{dist})', 'Location', 'best');

subplot(3,1,3);
plot(t_out*1000, x_response*1e6, 'k-', 'LineWidth', 2);
grid on;
xlabel('Time [ms]', 'FontSize', 11);
ylabel('Position [um]', 'FontSize', 11);
title('Part 1d: Rotor Position Response (0 RPM)', 'FontSize', 12);

saveas(gcf, 'part1d_amb_step_response.png');
fprintf('Plot saved: part1d_amb_step_response.png\n');

fprintf('\nStep Response Metrics:\n');
fprintf('  Peak displacement: %.4f um\n', max(abs(x_response))*1e6);
fprintf('  Final displacement: %.4f um\n', abs(x_response(end))*1e6);

%% ========================================================================
% DELIVERABLE 1e: DYNAMIC STIFFNESS
% =========================================================================
% Calculate the dynamic stiffness of the AMB system as a function of
% frequency for both radial and tilting directions.
%
% DEFINITION:
%   Dynamic Stiffness = |F/X| = |1 / (X/F_dist)|
%
% For closed-loop system:
%   X/F_dist = 1 / (m*s^2 - K_s + G_pos*K_i)
%   Dynamic Stiffness = |m*s^2 - K_s + G_pos*K_i|
%
% FREQUENCY RANGE: 1 Hz to 1000 Hz
%
% TILTING MODE:
%   - Uses tilting controller from Appendix B
%   - Effective inertia = I_transverse
%   - Two AMBs separated by L_amb
% =========================================================================

fprintf('\n================================================================\n');
fprintf('PART 1e: Dynamic Stiffness Analysis\n');
fprintf('================================================================\n\n');

% Frequency range
freq_range = logspace(0, 3, 200);  % 1 Hz to 1000 Hz
omega_range = 2*pi*freq_range;

% AMB separation (estimated from geometry)
L_amb_bl = baseline.flywheel_length + 0.3;  % ~1.3 m

% Transverse moment of inertia (for tilting)
I_transverse_bl = (1/12) * baseline.total_mass * (3*(baseline.flywheel_diameter/2)^2 + baseline.flywheel_length^2);

stiffness_radial = zeros(size(omega_range));
stiffness_tilt = zeros(size(omega_range));

fprintf('Computing dynamic stiffness from 1-1000 Hz...\n');

for idx = 1:length(omega_range)
    s_val = 1j * omega_range(idx);

    % Radial position controller
    G_cx_val = baseline.kpx + baseline.kix/s_val + s_val*baseline.kdx/(1 + s_val/baseline.omega_px);

    % Radial dynamic stiffness
    denom_radial = baseline.total_mass*s_val^2 + K_s_bl + G_cx_val*K_i_bl;
    stiffness_radial(idx) = abs(denom_radial);

    % Tilting controller
    G_alpha = baseline.kp_alpha + baseline.ki_alpha/s_val + s_val*baseline.kd_alpha/(1 + s_val/baseline.omega_p_alpha);

    % Tilting stiffness (effective parameters for two AMBs)
    K_s_tilt = K_s_bl * (L_amb_bl/2)^2 * 2;
    K_i_tilt = K_i_bl * (L_amb_bl/2) * 2;

    denom_tilt = I_transverse_bl*s_val^2 + K_s_tilt + G_alpha*K_i_tilt;
    stiffness_tilt(idx) = abs(denom_tilt) / (L_amb_bl/2)^2;  % Convert to linear stiffness
end

% === PLOT: Part 1e - Dynamic Stiffness ===
figure('Name', 'Part 1e: Dynamic Stiffness', 'Position', [100 100 900 450]);

loglog(freq_range, stiffness_radial/1e6, 'b-', 'LineWidth', 2.5); hold on;
loglog(freq_range, stiffness_tilt/1e6, 'r-', 'LineWidth', 2.5);
grid on;
xlabel('Frequency [Hz]', 'FontSize', 12);
ylabel('Dynamic Stiffness [MN/m]', 'FontSize', 12);
title('Part 1e: AMB Dynamic Stiffness vs Frequency', 'FontSize', 14);
legend('Radial Direction', 'Tilting Direction', 'Location', 'best', 'FontSize', 12);

saveas(gcf, 'part1e_dynamic_stiffness.png');
fprintf('Plot saved: part1e_dynamic_stiffness.png\n');

fprintf('\nDynamic Stiffness at Key Frequencies:\n');
fprintf('  Frequency    Radial       Tilting\n');
fprintf('  [Hz]         [MN/m]       [MN/m]\n');
fprintf('  ---------------------------------\n');
for f = [10, 100, 500, 1000]
    fprintf('  %4.0f         %.2f        %.2f\n', f, ...
        interp1(freq_range, stiffness_radial, f)/1e6, ...
        interp1(freq_range, stiffness_tilt, f)/1e6);
end

%% ========================================================================
% DELIVERABLE 1f: ROTOR RUNOUT
% =========================================================================
% Calculate rotor runout (displacement amplitude) due to mass imbalance
% as a function of state of charge.
%
% BALANCE GRADE:
%   ISO G2.5 (from Table 1 in project spec)
%   G = e * omega = 2.5 mm/s
%   where e = eccentricity [m]
%
% PROCESS:
%   1. Calculate eccentricity from balance grade: e = G / omega
%   2. Calculate unbalance force: F_unb = m * e * omega^2
%   3. Calculate dynamic stiffness at synchronous frequency
%   4. Runout = F_unb / Dynamic_Stiffness
%
% ASSUMPTIONS:
%   1. ISO G2.5 balance grade (typical for precision rotors)
%   2. Dynamic stiffness from closed-loop AMB system
%   3. Synchronous excitation at rotation frequency
% =========================================================================

fprintf('\n================================================================\n');
fprintf('PART 1f: Rotor Runout Analysis\n');
fprintf('================================================================\n\n');

G_grade = 2.5;  % ISO balance grade G2.5 [mm/s]
fprintf('Balance Grade: ISO G%.1f\n\n', G_grade);

% SoC range for analysis
SoC_runout = linspace(0, 100, 100);
runout_array = zeros(size(SoC_runout));
omega_runout = zeros(size(SoC_runout));

for idx = 1:length(SoC_runout)
    % Speed at this SoC
    omega = baseline.omega_min + (baseline.omega_max - baseline.omega_min) * SoC_runout(idx)/100;
    omega_runout(idx) = omega * 60 / (2*pi);  % RPM

    % Eccentricity from balance grade: e = G / omega
    e = (G_grade * 1e-3) / omega;  % [m]

    % Unbalance force
    F_unbalance = baseline.total_mass * e * omega^2;  % [N]

    % Dynamic stiffness at synchronous frequency
    s_val = 1j * omega;
    G_cx_val = baseline.kpx + baseline.kix/s_val + s_val*baseline.kdx/(1 + s_val/baseline.omega_px);
    dyn_stiff = abs(baseline.total_mass*s_val^2 + K_s_bl + G_cx_val*K_i_bl);

    % Runout = Force / Stiffness
    runout_array(idx) = F_unbalance / dyn_stiff * 1e6;  % [um]
end

% === PLOT: Part 1f - Rotor Runout ===
figure('Name', 'Part 1f: Rotor Runout', 'Position', [100 100 900 450]);

plot(SoC_runout, runout_array, 'b-', 'LineWidth', 2.5);
grid on;
xlabel('State of Charge [%]', 'FontSize', 12);
ylabel('Rotor Runout [um]', 'FontSize', 12);
title('Part 1f: Rotor Runout Due to Mass Imbalance (ISO G2.5)', 'FontSize', 14);

% Add secondary axis for RPM
ax1 = gca;
ax2 = axes('Position', ax1.Position, 'XAxisLocation', 'top', 'Color', 'none');
ax2.XLim = [baseline.min_speed_rpm baseline.max_speed_rpm]/1000;
ax2.YTick = [];
xlabel(ax2, 'Rotational Speed [krpm]', 'FontSize', 12);

saveas(gcf, 'part1f_rotor_runout.png');
fprintf('Plot saved: part1f_rotor_runout.png\n');

fprintf('Runout at Key SoC Points:\n');
fprintf('  SoC     Speed      Runout\n');
fprintf('  [%%]     [RPM]      [um]\n');
fprintf('  ------------------------\n');
fprintf('  %3.0f     %5.0f      %.3f\n', 0, baseline.min_speed_rpm, runout_array(1));
fprintf('  %3.0f     %5.0f      %.3f\n', 50, (baseline.min_speed_rpm+baseline.max_speed_rpm)/2, interp1(SoC_runout, runout_array, 50));
fprintf('  %3.0f     %5.0f      %.3f\n', 100, baseline.max_speed_rpm, runout_array(end));

%% ========================================================================
% DELIVERABLE 1 SUMMARY
% =========================================================================

fprintf('\n================================================================\n');
fprintf('DELIVERABLE 1 SUMMARY\n');
fprintf('================================================================\n\n');

fprintf('BASELINE SYSTEM SPECIFICATIONS:\n');
fprintf('  Flywheel: %.0f mm dia x %.0f mm length\n', ...
    baseline.flywheel_diameter*1000, baseline.flywheel_length*1000);
fprintf('  Total mass: %.1f kg\n', baseline.total_mass);
fprintf('  Moment of inertia: %.3f kg*m^2\n\n', baseline.I_total);

fprintf('PERFORMANCE METRICS:\n');
fprintf('  1a. Max losses: %.2f kW, Max temp: %.1f C\n', max(total_loss_array)/1000, max(temperature_array));
fprintf('  1b. Specific power: %.3f kW/kg\n', specific_power_bl);
fprintf('  1b. Specific energy: %.2f Wh/kg\n', specific_energy_bl);
fprintf('  1c. Cycle efficiency: %.2f%%\n', efficiency_1c);
fprintf('  1d. Peak AMB displacement: %.3f um\n', max(abs(x_response))*1e6);
fprintf('  1e. Dynamic stiffness at 100 Hz: %.2f MN/m\n', interp1(freq_range, stiffness_radial, 100)/1e6);
fprintf('  1f. Max runout: %.3f um\n', max(runout_array));

%% ########################################################################
%                         DELIVERABLE 2
%                    NEW DESIGN DEVELOPMENT
% #########################################################################

fprintf('\n\n');
fprintf('================================================================\n');
fprintf('              DELIVERABLE 2: DESIGN STUDY\n');
fprintf('================================================================\n\n');

%% ========================================================================
% DELIVERABLE 2a: DESIGN PARAMETER STUDY
% =========================================================================
% Explore the design space by varying magnet thickness and maximum speed
% to find designs that optimize specific power, specific energy, and
% efficiency for the Team 16 6-hour storage cycle.
%
% DESIGN VARIABLES:
%   - Magnet thickness: 2-10 mm
%   - Maximum speed: 20,000-60,000 RPM
%
% DERIVED PARAMETERS (from tip speed limits):
%   - Shaft diameter: limited by steel/PM tip speed (175 m/s)
%   - Flywheel diameter: limited by composite tip speed (900 m/s)
%
% CONSTRAINTS:
%   - Max temperature: 100 C
%   - SoC must stay > 0% during cycle
%   - Rated power must meet cycle demand (~430 kW peak for Team 16)
%
% OBJECTIVES:
%   - Maximize specific power [kW/kg]
%   - Maximize specific energy [Wh/kg]
%   - Maximize cycle efficiency [%]
%   - Minimize temperature [C]
% =========================================================================

fprintf('================================================================\n');
fprintf('PART 2a: Design Parameter Study\n');
fprintf('================================================================\n\n');

% Design variable ranges
magnet_thickness_range = linspace(0.002, 0.010, 9);   % 2-10 mm
max_speed_range = linspace(20000, 60000, 9);          % 20k-60k RPM

n_mag = length(magnet_thickness_range);
n_speed = length(max_speed_range);

% Results storage
results = struct();
results.magnet_thickness = zeros(n_mag, n_speed);
results.max_speed_rpm = zeros(n_mag, n_speed);
results.specific_power = zeros(n_mag, n_speed);
results.specific_energy = zeros(n_mag, n_speed);
results.efficiency = zeros(n_mag, n_speed);
results.max_temp = zeros(n_mag, n_speed);
results.viable = zeros(n_mag, n_speed);
results.flywheel_diameter = zeros(n_mag, n_speed);
results.flywheel_length = zeros(n_mag, n_speed);
results.shaft_diameter = zeros(n_mag, n_speed);
results.motor_length = zeros(n_mag, n_speed);
results.total_mass = zeros(n_mag, n_speed);
results.rated_power = zeros(n_mag, n_speed);
results.min_soc = zeros(n_mag, n_speed);

fprintf('Evaluating %d x %d = %d design points...\n', n_mag, n_speed, n_mag*n_speed);
fprintf('Progress: ');

for i = 1:n_mag
    for j = 1:n_speed
        t_mag = magnet_thickness_range(i);
        omega_max_rpm = max_speed_range(j);
        omega_max = omega_max_rpm * 2*pi / 60;
        omega_min = omega_max / 2;  % 0% SoC at half speed

        results.magnet_thickness(i,j) = t_mag;
        results.max_speed_rpm(i,j) = omega_max_rpm;

        % === STEP 1: Shaft diameter from tip speed limits ===
        % PM outer radius = r_shaft + t_mag
        % PM tip speed = (r_shaft + t_mag) * omega_max <= 175 m/s
        r_shaft_pm = (max_pm_tip_speed / omega_max) - t_mag;
        r_shaft_steel = max_steel_tip_speed / omega_max;
        r_shaft = min(r_shaft_pm, r_shaft_steel);

        if r_shaft < 0.020  % Min 20 mm radius
            results.viable(i,j) = 0;
            continue;
        end
        d_shaft = 2 * r_shaft;
        results.shaft_diameter(i,j) = d_shaft;

        % === STEP 2: Flywheel diameter from composite tip speed ===
        r_flywheel = max_composite_tip_speed / omega_max;
        d_flywheel = 2 * r_flywheel;

        if d_flywheel < d_shaft + 0.050  % Min 25mm wall thickness
            results.viable(i,j) = 0;
            continue;
        end
        results.flywheel_diameter(i,j) = d_flywheel;

        % === STEP 3: Motor sizing for required power ===
        % Team 16 cycle peaks at ~430 kW, target 450 kW
        shear = magneticShear(t_mag, I_rated_pu);
        torque_per_length = shear * pi * d_shaft * (d_shaft/2);
        L_motor_min = 450000 / (torque_per_length * omega_max);
        L_motor = max(0.150, min(0.600, ceil(L_motor_min * 100) / 100));
        results.motor_length(i,j) = L_motor;

        motor_area_new = pi * d_shaft * L_motor;
        P_rated = shear * motor_area_new * (d_shaft/2) * omega_max;
        results.rated_power(i,j) = P_rated;

        % === STEP 4: Flywheel length for required energy ===
        % Target 40 kWh capacity for Team 16 cycle
        E_target = 40e3 * 3600;  % [J]
        r_outer = d_flywheel / 2;
        r_inner = d_shaft / 2;

        V_per_length = pi * (r_outer^2 - r_inner^2);
        m_per_length = rho_composite * V_per_length;
        I_per_length = 0.5 * m_per_length * (r_outer^2 + r_inner^2);

        I_required = 2 * E_target / (omega_max^2 - omega_min^2);
        L_flywheel_min = I_required / I_per_length;
        L_flywheel = max(0.500, min(4.000, ceil(L_flywheel_min * 10) / 10));
        results.flywheel_length(i,j) = L_flywheel;

        % === STEP 5: Mass and inertia ===
        V_flywheel = pi * (r_outer^2 - r_inner^2) * L_flywheel;
        m_flywheel = rho_composite * V_flywheel;

        L_shaft = L_flywheel + L_motor + 0.38;  % Extra for AMBs, clearances
        V_shaft = pi * r_inner^2 * L_shaft;
        m_shaft = rho_steel * V_shaft;

        r_mag = r_inner + t_mag;
        V_magnet = pi * (r_mag^2 - r_inner^2) * L_motor;
        m_magnet = rho_magnet * V_magnet;

        m_amb = 0.10 * m_shaft;
        m_total = m_flywheel + m_shaft + m_magnet + m_amb;
        results.total_mass(i,j) = m_total;

        I_flywheel = 0.5 * m_flywheel * (r_outer^2 + r_inner^2);
        I_shaft = 0.5 * m_shaft * r_inner^2;
        I_magnet = 0.5 * m_magnet * (r_mag^2 + r_inner^2);
        I_total = I_flywheel + I_shaft + I_magnet;

        % === STEP 6: Specific power and energy ===
        E_max = 0.5 * I_total * omega_max^2;
        E_min = 0.5 * I_total * omega_min^2;
        E_stored = E_max - E_min;

        sp = (P_rated / 1000) / m_total;
        se = (E_stored / 3600) / m_total;
        results.specific_power(i,j) = sp;
        results.specific_energy(i,j) = se;

        % === STEP 7: Cycle simulation ===
        dt_sim = 10.0;
        t_sim = 0:dt_sim:21600;  % 6-hour cycle

        omega_curr = (omega_max + omega_min) / 2;
        E_in_cycle = 0;
        E_out_cycle = 0;
        min_soc = 50;
        max_temp_cycle = 20;

        A_surface = 2*pi*r_outer*L_flywheel + 2*pi*r_outer^2;
        % Correct two-surface enclosure radiation model (consistent with Part 1a)
        housing_clearance_2a = 0.020;  % 20mm radial clearance
        housing_inner_dia_2a = d_flywheel + 2*housing_clearance_2a;
        A_housing_2a = 2*pi*(housing_inner_dia_2a/2)*L_flywheel + 2*pi*(housing_inner_dia_2a/2)^2;
        rad_factor_2a = 1/epsilon_rotor + (A_surface/A_housing_2a)*(1/epsilon_housing - 1);

        for k = 1:length(t_sim)
            P_grid = team_16_cycle(t_sim(k));

            SoC = 100 * (omega_curr - omega_min) / (omega_max - omega_min);
            if SoC < min_soc
                min_soc = SoC;
            end

            omega_rpm_curr = omega_curr * 60 / (2*pi);
            I_pu = min(1.0, I_rated_pu * abs(P_grid) / P_rated);

            P_rotor = rotorLosses(t_mag, d_shaft, L_motor, I_pu, omega_rpm_curr);
            P_stator = statorLosses(t_mag, d_shaft, L_motor, I_pu, omega_rpm_curr);
            P_loss = P_rotor + P_stator;

            T_rotor = (T_housing_K^4 + P_rotor*rad_factor_2a/(sigma*A_surface))^0.25;
            T_celsius = T_rotor - 273;
            if T_celsius > max_temp_cycle
                max_temp_cycle = T_celsius;
            end

            E_curr = 0.5 * I_total * omega_curr^2;
            dE = -(P_grid + P_loss) * dt_sim;
            E_curr = E_curr + dE;

            if E_curr > 0
                omega_curr = sqrt(2 * E_curr / I_total);
            end
            omega_curr = max(omega_min*0.9, min(omega_max*1.05, omega_curr));

            if P_grid > 0
                E_out_cycle = E_out_cycle + P_grid * dt_sim;
            else
                E_in_cycle = E_in_cycle + abs(P_grid) * dt_sim;
            end
        end

        results.min_soc(i,j) = min_soc;
        results.max_temp(i,j) = max_temp_cycle;

        % Efficiency calculation
        E_recovery = max(0, 0.5*I_total*((omega_max+omega_min)/2)^2 - 0.5*I_total*omega_curr^2);
        if (E_in_cycle + E_recovery) > 0
            eff = (E_out_cycle / (E_in_cycle + E_recovery)) * 100;
        else
            eff = 0;
        end
        results.efficiency(i,j) = eff;

        % === STEP 8: Viability check ===
        viable = 1;
        if max_temp_cycle > max_temp, viable = 0; end
        if min_soc < 0, viable = 0; end
        if eff < 70 || eff > 105, viable = 0; end
        if P_rated < 430000, viable = 0; end
        results.viable(i,j) = viable;
    end
    fprintf('.');
end
fprintf(' Done!\n\n');

% Count viable designs
n_viable = sum(results.viable(:));
fprintf('Viable designs: %d out of %d\n\n', n_viable, n_mag*n_speed);

%% ========================================================================
% DELIVERABLE 2a: SELECT OPTIMAL DESIGN
% =========================================================================
% Use weighted multi-objective optimization to select the best design.
%
% SCORING:
%   - Normalize each objective to 0-1 range
%   - Weights: 30% specific power, 30% specific energy, 25% efficiency, 15% temperature
%   - Select design with highest composite score
% =========================================================================

fprintf('Selecting optimal design using weighted objectives...\n');
fprintf('  Weights: 30%% SP, 30%% SE, 25%% Eff, 15%% Temp\n\n');

% Normalize objectives
sp_norm = (results.specific_power - min(results.specific_power(:))) / ...
    (max(results.specific_power(:)) - min(results.specific_power(:)) + eps);
se_norm = (results.specific_energy - min(results.specific_energy(:))) / ...
    (max(results.specific_energy(:)) - min(results.specific_energy(:)) + eps);
eff_norm = (results.efficiency - 90) / 10;
temp_norm = 1 - (results.max_temp - 20) / 80;

score = 0.3*sp_norm + 0.3*se_norm + 0.25*eff_norm + 0.15*temp_norm;
score(results.viable == 0) = -Inf;

[~, best_idx] = max(score(:));
[best_i, best_j] = ind2sub(size(score), best_idx);

% Extract optimal design
optimal = struct();
optimal.magnet_thickness = results.magnet_thickness(best_i, best_j);
optimal.max_speed_rpm = results.max_speed_rpm(best_i, best_j);
optimal.flywheel_diameter = results.flywheel_diameter(best_i, best_j);
optimal.flywheel_length = results.flywheel_length(best_i, best_j);
optimal.shaft_diameter = results.shaft_diameter(best_i, best_j);
optimal.motor_length = results.motor_length(best_i, best_j);
optimal.total_mass = results.total_mass(best_i, best_j);
optimal.specific_power = results.specific_power(best_i, best_j);
optimal.specific_energy = results.specific_energy(best_i, best_j);
optimal.efficiency = results.efficiency(best_i, best_j);
optimal.max_temp = results.max_temp(best_i, best_j);
optimal.rated_power = results.rated_power(best_i, best_j);
optimal.min_soc = results.min_soc(best_i, best_j);

fprintf('OPTIMAL DESIGN SPECIFICATIONS:\n');
fprintf('  Design Variables:\n');
fprintf('    Magnet thickness: %.1f mm\n', optimal.magnet_thickness*1000);
fprintf('    Max speed: %.0f RPM\n', optimal.max_speed_rpm);
fprintf('  Derived Dimensions:\n');
fprintf('    Flywheel diameter: %.1f mm\n', optimal.flywheel_diameter*1000);
fprintf('    Flywheel length: %.1f mm\n', optimal.flywheel_length*1000);
fprintf('    Shaft diameter: %.1f mm\n', optimal.shaft_diameter*1000);
fprintf('    Motor length: %.1f mm\n', optimal.motor_length*1000);
fprintf('  Performance:\n');
fprintf('    Total mass: %.1f kg\n', optimal.total_mass);
fprintf('    Rated power: %.1f kW\n', optimal.rated_power/1000);
fprintf('    Specific power: %.3f kW/kg\n', optimal.specific_power);
fprintf('    Specific energy: %.2f Wh/kg\n', optimal.specific_energy);
fprintf('    Cycle efficiency: %.2f%%\n', optimal.efficiency);
fprintf('    Max temperature: %.1f C\n', optimal.max_temp);
fprintf('    Min SoC: %.1f%%\n\n', optimal.min_soc);

% === PLOT: Part 2a - Design Trade-offs ===
figure('Name', 'Part 2a: Design Trade-offs', 'Position', [100 100 1200 500]);

subplot(1,2,1);
viable_mask = results.viable == 1;
scatter(results.specific_energy(viable_mask), results.specific_power(viable_mask), ...
    80, results.efficiency(viable_mask), 'filled');
colorbar;
colormap(gca, 'parula');
hold on;
plot(specific_energy_bl, specific_power_bl, 'rp', 'MarkerSize', 15, 'MarkerFaceColor', 'r');
plot(optimal.specific_energy, optimal.specific_power, 'gs', 'MarkerSize', 15, 'MarkerFaceColor', 'g');
grid on;
xlabel('Specific Energy [Wh/kg]', 'FontSize', 12);
ylabel('Specific Power [kW/kg]', 'FontSize', 12);
title('Part 2a: Design Trade-offs (color = efficiency)', 'FontSize', 14);
legend('Viable Designs', 'Baseline', 'Optimal', 'Location', 'best');

subplot(1,2,2);
[MAG, SPD] = meshgrid(magnet_thickness_range*1000, max_speed_range/1000);
eff_plot = results.efficiency';
eff_plot(results.viable' == 0) = NaN;
contourf(MAG, SPD, eff_plot, 20);
colorbar;
xlabel('Magnet Thickness [mm]', 'FontSize', 12);
ylabel('Max Speed [krpm]', 'FontSize', 12);
title('Part 2a: Cycle Efficiency [%] (viable only)', 'FontSize', 14);

saveas(gcf, 'part2a_design_tradeoffs.png');
fprintf('Plot saved: part2a_design_tradeoffs.png\n');

%% ========================================================================
% DELIVERABLE 2a: COMPARISON TO BASELINE
% =========================================================================

fprintf('\n================================================================\n');
fprintf('PART 2a: Comparison to Baseline\n');
fprintf('================================================================\n\n');

fprintf('                        Baseline    New Design    Change\n');
fprintf('------------------------------------------------------------\n');
fprintf('Specific Power [kW/kg]   %.3f       %.3f        %+.1f%%\n', ...
    specific_power_bl, optimal.specific_power, ...
    100*(optimal.specific_power - specific_power_bl)/specific_power_bl);
fprintf('Specific Energy [Wh/kg]  %.2f       %.2f        %+.1f%%\n', ...
    specific_energy_bl, optimal.specific_energy, ...
    100*(optimal.specific_energy - specific_energy_bl)/specific_energy_bl);
fprintf('Cycle Efficiency [%%]     %.2f       %.2f        %+.1f\n', ...
    efficiency_1c, optimal.efficiency, optimal.efficiency - efficiency_1c);
fprintf('Max Temperature [C]      %.1f        %.1f        %+.1f\n', ...
    max(temperature_array), optimal.max_temp, optimal.max_temp - max(temperature_array));

%% ========================================================================
% DELIVERABLE 2b: DETAILED CYCLE SIMULATION
% =========================================================================
% Run a detailed simulation of the optimal design through the Team 16
% 6-hour storage cycle with fine time resolution for plotting.
%
% OUTPUTS:
%   - Power demand profile
%   - SoC vs time
%   - Losses vs time
%   - Temperature vs time
% =========================================================================

fprintf('\n================================================================\n');
fprintf('PART 2b: Optimal Design Cycle Simulation\n');
fprintf('================================================================\n\n');

% Fine time resolution for plotting
dt_fine = 5.0;
t_fine = 0:dt_fine:21600;

% Recalculate optimal design inertia
r_outer_opt = optimal.flywheel_diameter / 2;
r_inner_opt = optimal.shaft_diameter / 2;
r_mag_opt = r_inner_opt + optimal.magnet_thickness;

V_fw_opt = pi * (r_outer_opt^2 - r_inner_opt^2) * optimal.flywheel_length;
m_fw_opt = rho_composite * V_fw_opt;
L_sh_opt = optimal.flywheel_length + optimal.motor_length + 0.38;
V_sh_opt = pi * r_inner_opt^2 * L_sh_opt;
m_sh_opt = rho_steel * V_sh_opt;
V_mg_opt = pi * (r_mag_opt^2 - r_inner_opt^2) * optimal.motor_length;
m_mg_opt = rho_magnet * V_mg_opt;

I_fw_opt = 0.5 * m_fw_opt * (r_outer_opt^2 + r_inner_opt^2);
I_sh_opt = 0.5 * m_sh_opt * r_inner_opt^2;
I_mg_opt = 0.5 * m_mg_opt * (r_mag_opt^2 + r_inner_opt^2);
I_total_opt = I_fw_opt + I_sh_opt + I_mg_opt;

omega_max_opt = optimal.max_speed_rpm * 2*pi / 60;
omega_min_opt = omega_max_opt / 2;

A_surf_opt = 2*pi*r_outer_opt*optimal.flywheel_length + 2*pi*r_outer_opt^2;
shear_opt = magneticShear(optimal.magnet_thickness, I_rated_pu);
P_rated_opt = shear_opt * pi * optimal.shaft_diameter * optimal.motor_length * (optimal.shaft_diameter/2) * omega_max_opt;

% Simulation arrays
omega_opt = zeros(size(t_fine));
SoC_opt = zeros(size(t_fine));
power_grid_opt = zeros(size(t_fine));
power_loss_opt = zeros(size(t_fine));
temp_opt = zeros(size(t_fine));

% Correct two-surface enclosure radiation model (consistent with Part 1a)
housing_clearance_2b = 0.020;  % 20mm radial clearance
housing_inner_dia_2b = optimal.flywheel_diameter + 2*housing_clearance_2b;
A_housing_2b = 2*pi*(housing_inner_dia_2b/2)*optimal.flywheel_length + 2*pi*(housing_inner_dia_2b/2)^2;
rad_factor_2b = 1/epsilon_rotor + (A_surf_opt/A_housing_2b)*(1/epsilon_housing - 1);
omega_curr = (omega_max_opt + omega_min_opt) / 2;

fprintf('Running detailed 6-hour cycle simulation...\n');

for k = 1:length(t_fine)
    P_grid = team_16_cycle(t_fine(k));
    power_grid_opt(k) = P_grid;

    omega_rpm = omega_curr * 60 / (2*pi);
    SoC_opt(k) = 100 * (omega_curr - omega_min_opt) / (omega_max_opt - omega_min_opt);
    omega_opt(k) = omega_curr;

    I_pu = min(1.0, I_rated_pu * abs(P_grid) / P_rated_opt);

    P_rotor = rotorLosses(optimal.magnet_thickness, optimal.shaft_diameter, ...
        optimal.motor_length, I_pu, omega_rpm);
    P_stator = statorLosses(optimal.magnet_thickness, optimal.shaft_diameter, ...
        optimal.motor_length, I_pu, omega_rpm);
    power_loss_opt(k) = P_rotor + P_stator;

    T_rotor = (T_housing_K^4 + P_rotor*rad_factor_2b/(sigma*A_surf_opt))^0.25;
    temp_opt(k) = T_rotor - 273;

    E_curr = 0.5 * I_total_opt * omega_curr^2;
    dE = -(P_grid + P_rotor + P_stator) * dt_fine;
    E_curr = E_curr + dE;

    if E_curr > 0
        omega_curr = sqrt(2 * E_curr / I_total_opt);
    end
    omega_curr = max(omega_min_opt*0.9, min(omega_max_opt*1.05, omega_curr));
end

% === PLOT: Part 2b - Optimal Design Cycle ===
figure('Name', 'Part 2b: Optimal Design Cycle', 'Position', [100 100 1000 800]);

subplot(2,2,1);
plot(t_fine/3600, power_grid_opt/1000, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time [hours]', 'FontSize', 11);
ylabel('Grid Power [kW]', 'FontSize', 11);
title('Part 2b: Team 16 Cycle Power Demand', 'FontSize', 12);

subplot(2,2,2);
plot(t_fine/3600, SoC_opt, 'r-', 'LineWidth', 2);
hold on;
yline(0, 'k--', 'LineWidth', 1.5);
grid on;
xlabel('Time [hours]', 'FontSize', 11);
ylabel('State of Charge [%]', 'FontSize', 11);
title('Part 2b: SoC During Cycle', 'FontSize', 12);
ylim([-5 105]);

subplot(2,2,3);
plot(t_fine/3600, power_loss_opt/1000, 'k-', 'LineWidth', 1.5);
grid on;
xlabel('Time [hours]', 'FontSize', 11);
ylabel('Total Losses [kW]', 'FontSize', 11);
title('Part 2b: Motor Losses', 'FontSize', 12);

subplot(2,2,4);
plot(t_fine/3600, temp_opt, 'm-', 'LineWidth', 2);
hold on;
yline(100, 'r--', 'LineWidth', 1.5, 'Label', 'Max Safe');
grid on;
xlabel('Time [hours]', 'FontSize', 11);
ylabel('Temperature [C]', 'FontSize', 11);
title('Part 2b: Rotor Temperature', 'FontSize', 12);

saveas(gcf, 'part2b_optimal_cycle.png');
fprintf('Plot saved: part2b_optimal_cycle.png\n');

fprintf('\nCycle Summary:\n');
fprintf('  Min SoC: %.1f%%\n', min(SoC_opt));
fprintf('  Max SoC: %.1f%%\n', max(SoC_opt));
fprintf('  Max temperature: %.1f C\n', max(temp_opt));
fprintf('  Max losses: %.2f kW\n', max(power_loss_opt)/1000);

%% ########################################################################
%                         DELIVERABLE 3
%                    AMB CONTROLLER DESIGN
% #########################################################################

fprintf('\n\n');
fprintf('================================================================\n');
fprintf('              DELIVERABLE 3: AMB CONTROLLER DESIGN\n');
fprintf('================================================================\n\n');

%% ========================================================================
% DELIVERABLE 3a: CONTROLLER TRANSFER FUNCTIONS
% =========================================================================
% Design AMB controllers for the new optimized flywheel design.
%
% APPROACH:
%   Scale the baseline controller gains to account for:
%   - Different rotor mass
%   - Different AMB force constant
%   - Different AMB stiffness
%
% CONTROLLER STRUCTURE (from Appendix B):
%   Current controller: G_ci(s) = Kp + Ki/s = 345 + 2149/s
%   Position controller: G_pos(s) = kp + ki/s + kd*s/(1+s/wp)
%
% SCALING RULES:
%   - Gains scale with mass ratio and inverse of force constant ratio
%   - Filter frequencies kept constant
% =========================================================================

fprintf('================================================================\n');
fprintf('PART 3a: Controller Transfer Functions\n');
fprintf('================================================================\n\n');

% New system AMB sizing
% AMB rated force = 2 x rotating group weight
g = 9.81;
new_amb_force = 2 * optimal.total_mass * g;

fprintf('AMB Sizing:\n');
fprintf('  Baseline mass: %.1f kg, AMB force: %.0f N\n', baseline.total_mass, baseline.amb_rated_force);
fprintf('  New mass: %.1f kg, AMB force: %.0f N\n\n', optimal.total_mass, new_amb_force);

% Get AMB parameters
amb_new = ambParameters(optimal.shaft_diameter, new_amb_force);

K_s_new = -amb_new.stiffnessConstant;  % Negative stiffness [N/m]
K_i_new = amb_new.forceConstant;        % Force constant [N/A]

fprintf('AMB Parameters Comparison:\n');
fprintf('                        Baseline        New Design\n');
fprintf('  Stiffness [N/m]:      %.2e    %.2e\n', K_s_bl, K_s_new);
fprintf('  Force constant [N/A]: %.2f          %.2f\n', K_i_bl, K_i_new);
fprintf('  Inductance [H]:       %.4f          %.4f\n', amb_bl.coilInductance, amb_new.coilInductance);
fprintf('  Resistance [Ohm]:     %.3f           %.3f\n\n', amb_bl.coilResistance, amb_new.coilResistance);

% Scaling factors
mass_ratio = optimal.total_mass / baseline.total_mass;
Ki_ratio = K_i_new / K_i_bl;

fprintf('Scaling Factors:\n');
fprintf('  Mass ratio: %.3f\n', mass_ratio);
fprintf('  Ki ratio: %.3f\n\n', Ki_ratio);

% New position controller X (scaled from baseline)
new_ctrl = struct();
new_ctrl.kpx = baseline.kpx * mass_ratio / Ki_ratio;
new_ctrl.kix = baseline.kix * mass_ratio / Ki_ratio;
new_ctrl.kdx = baseline.kdx * mass_ratio / Ki_ratio;
new_ctrl.omega_px = baseline.omega_px;

% New tilting controller (scale by inertia ratio)
L_amb_new = optimal.flywheel_length + 0.3;
I_ratio = I_total_opt / baseline.I_total;

new_ctrl.kp_alpha = baseline.kp_alpha * I_ratio / Ki_ratio;
new_ctrl.ki_alpha = baseline.ki_alpha * I_ratio / Ki_ratio;
new_ctrl.kd_alpha = baseline.kd_alpha * I_ratio / Ki_ratio;
new_ctrl.omega_p_alpha = baseline.omega_p_alpha;

% Print transfer functions
fprintf('CONTROLLER TRANSFER FUNCTIONS:\n\n');

fprintf('1. Current Controller (same for both):\n');
fprintf('   G_ci(s) = %.0f + %.0f/s\n\n', baseline.Kp_current, baseline.Ki_current);

fprintf('2. Position Controller X - Baseline:\n');
fprintf('   G_pos(s) = kp + ki/s + kd*s/(1+s/wp)\n');
fprintf('   kp = %.4e N/m\n', baseline.kpx);
fprintf('   ki = %.5e N/(m*s)\n', baseline.kix);
fprintf('   kd = %.0f N*s/m\n', baseline.kdx);
fprintf('   wp = %.0f rad/s\n\n', baseline.omega_px);

fprintf('3. Position Controller X - New Design:\n');
fprintf('   kp = %.4e N/m\n', new_ctrl.kpx);
fprintf('   ki = %.5e N/(m*s)\n', new_ctrl.kix);
fprintf('   kd = %.0f N*s/m\n', new_ctrl.kdx);
fprintf('   wp = %.0f rad/s\n\n', new_ctrl.omega_px);

fprintf('4. Tilting Controller - Baseline:\n');
fprintf('   kp = %.4e N*m/rad\n', baseline.kp_alpha);
fprintf('   ki = %.5e N*m/(rad*s)\n', baseline.ki_alpha);
fprintf('   kd = %.0f N*m*s/rad\n', baseline.kd_alpha);
fprintf('   wp = %.0f rad/s\n\n', baseline.omega_p_alpha);

fprintf('5. Tilting Controller - New Design:\n');
fprintf('   kp = %.4e N*m/rad\n', new_ctrl.kp_alpha);
fprintf('   ki = %.5e N*m/(rad*s)\n', new_ctrl.ki_alpha);
fprintf('   kd = %.0f N*m*s/rad\n', new_ctrl.kd_alpha);
fprintf('   wp = %.0f rad/s\n\n', new_ctrl.omega_p_alpha);

%% ========================================================================
% DELIVERABLE 3b: AMB STEP RESPONSE COMPARISON
% =========================================================================
% Compare the step response of baseline and new system AMBs using
% transfer function analysis (same approach as Part 1d).
%
% TRANSFER FUNCTION APPROACH:
%   - Plant: G_p(s) = K_i / (m*s^2 - K_s)  [note: K_s < 0]
%   - Position controller: G_pos(s) = kp + ki/s + kd*s/(1+s/wp)
%   - Closed-loop from disturbance: T_dist = G_dist / (1 + L)
%   - Current loop assumed fast (G_ci ≈ 1)
%
% METRICS:
%   - Peak displacement
%   - Settling time (2% criterion)
% =========================================================================

fprintf('================================================================\n');
fprintf('PART 3b: AMB Step Response Comparison\n');
fprintf('================================================================\n\n');

% Step disturbances: 10% of rated force
F_step_bl = 0.10 * baseline.amb_rated_force;
F_step_new = 0.10 * new_amb_force;

fprintf('Step disturbances:\n');
fprintf('  Baseline: %.1f N (10%% of %.0f N)\n', F_step_bl, baseline.amb_rated_force);
fprintf('  New: %.1f N (10%% of %.0f N)\n\n', F_step_new, new_amb_force);

% Time vector for simulation
t_step = linspace(0, 0.05, 5000);  % 50 ms

% === BASELINE Transfer Function Model ===
s = tf('s');

% Position controller: G_pos(s) = kp + ki/s + kd*s/(1+s/wp)
G_pos_bl = baseline.kpx + baseline.kix/s + s*baseline.kdx/(1 + s/baseline.omega_px);

% Plant: G_p(s) = K_i / (m*s^2 - K_s)
% Note: K_s_bl is already negative (destabilizing)
G_plant_bl = K_i_bl / (baseline.total_mass*s^2 + K_s_bl);

% Open-loop transfer function
L_bl = G_pos_bl * G_plant_bl;

% Disturbance transfer function (force to position without controller)
G_dist_bl = 1 / (baseline.total_mass*s^2 + K_s_bl);

% Closed-loop from disturbance to position
T_dist_bl = G_dist_bl / (1 + L_bl);

% Closed-loop from disturbance to control effort (for current calculation)
T_ctrl_bl = -L_bl * G_dist_bl / (1 + L_bl);

% Simulate step response
[x_step_bl, ~] = step(T_dist_bl * F_step_bl, t_step);
[f_ctrl_bl, ~] = step(T_ctrl_bl * F_step_bl * K_i_bl, t_step);
i_step_bl = f_ctrl_bl / K_i_bl;

% === NEW DESIGN Transfer Function Model ===
% Position controller for new design
G_pos_new = new_ctrl.kpx + new_ctrl.kix/s + s*new_ctrl.kdx/(1 + s/new_ctrl.omega_px);

% Plant for new design
G_plant_new = K_i_new / (optimal.total_mass*s^2 + K_s_new);

% Open-loop
L_new = G_pos_new * G_plant_new;

% Disturbance transfer function
G_dist_new = 1 / (optimal.total_mass*s^2 + K_s_new);

% Closed-loop from disturbance to position
T_dist_new = G_dist_new / (1 + L_new);

% Closed-loop from disturbance to control effort
T_ctrl_new = -L_new * G_dist_new / (1 + L_new);

% Simulate step response
[x_step_new, ~] = step(T_dist_new * F_step_new, t_step);
[f_ctrl_new, ~] = step(T_ctrl_new * F_step_new * K_i_new, t_step);
i_step_new = f_ctrl_new / K_i_new;

% === Calculate Metrics ===
peak_bl = max(abs(x_step_bl)) * 1e6;  % Convert to um
peak_new = max(abs(x_step_new)) * 1e6;

% Settling time (2% of peak)
ss_bl = x_step_bl(end);
threshold_bl = 0.02 * max(abs(x_step_bl));
settle_idx_bl = find(abs(x_step_bl - ss_bl) > threshold_bl, 1, 'last');
if isempty(settle_idx_bl), settle_idx_bl = 1; end
settle_bl = t_step(settle_idx_bl) * 1000;  % Convert to ms

ss_new = x_step_new(end);
threshold_new = 0.02 * max(abs(x_step_new));
settle_idx_new = find(abs(x_step_new - ss_new) > threshold_new, 1, 'last');
if isempty(settle_idx_new), settle_idx_new = 1; end
settle_new = t_step(settle_idx_new) * 1000;

% === PLOT: Part 3b - Step Response Comparison ===
figure('Name', 'Part 3b: Step Response', 'Position', [100 100 1000 600]);

subplot(2,2,1);
plot(t_step*1000, x_step_bl*1e6, 'b-', 'LineWidth', 2); hold on;
plot(t_step*1000, x_step_new*1e6, 'r--', 'LineWidth', 2);
grid on;
xlabel('Time [ms]', 'FontSize', 11);
ylabel('Position [um]', 'FontSize', 11);
title('Part 3b: Position Response', 'FontSize', 12);
legend('Baseline', 'New Design', 'Location', 'best');

subplot(2,2,2);
plot(t_step*1000, i_step_bl, 'b-', 'LineWidth', 2); hold on;
plot(t_step*1000, i_step_new, 'r--', 'LineWidth', 2);
grid on;
xlabel('Time [ms]', 'FontSize', 11);
ylabel('Current [A]', 'FontSize', 11);
title('Part 3b: Current Response', 'FontSize', 12);
legend('Baseline', 'New Design', 'Location', 'best');

subplot(2,2,3);
plot(t_step*1000, x_step_bl*1e6, 'b-', 'LineWidth', 2); hold on;
plot(t_step*1000, x_step_new*1e6, 'r--', 'LineWidth', 2);
xlim([0 10]);
grid on;
xlabel('Time [ms]', 'FontSize', 11);
ylabel('Position [um]', 'FontSize', 11);
title('Part 3b: Position (First 10 ms)', 'FontSize', 12);
legend('Baseline', 'New Design', 'Location', 'best');

subplot(2,2,4);
axis off;
text(0.1, 0.9, 'STEP RESPONSE METRICS', 'FontSize', 14, 'FontWeight', 'bold');
text(0.1, 0.7, sprintf('Peak Displacement:'), 'FontSize', 12, 'FontWeight', 'bold');
text(0.1, 0.55, sprintf('  Baseline: %.4f um', peak_bl), 'FontSize', 11);
text(0.1, 0.4, sprintf('  New: %.4f um', peak_new), 'FontSize', 11);
text(0.1, 0.25, sprintf('Settling Time (2%%):'), 'FontSize', 12, 'FontWeight', 'bold');
text(0.1, 0.1, sprintf('  Baseline: %.2f ms', settle_bl), 'FontSize', 11);
text(0.1, -0.05, sprintf('  New: %.2f ms', settle_new), 'FontSize', 11);

saveas(gcf, 'part3b_step_response.png');
fprintf('Plot saved: part3b_step_response.png\n');

fprintf('\nStep Response Comparison:\n');
fprintf('                      Baseline    New Design\n');
fprintf('  Peak displacement:  %.3f um    %.3f um\n', peak_bl, peak_new);
fprintf('  Settling time:      %.2f ms     %.2f ms\n', settle_bl, settle_new);

%% ========================================================================
% DELIVERABLE 3c: DYNAMIC STIFFNESS COMPARISON
% =========================================================================
% Compare dynamic stiffness between baseline and new design across
% frequency range 1-1000 Hz.
% =========================================================================

fprintf('\n================================================================\n');
fprintf('PART 3c: Dynamic Stiffness Comparison\n');
fprintf('================================================================\n\n');

stiff_bl_3c = zeros(size(omega_range));
stiff_new_3c = zeros(size(omega_range));

for idx = 1:length(omega_range)
    s_val = 1j * omega_range(idx);

    % Baseline
    G_bl = baseline.kpx + baseline.kix/s_val + baseline.kdx*s_val/(1+s_val/baseline.omega_px);
    stiff_bl_3c(idx) = abs(baseline.total_mass*s_val^2 + K_s_bl + G_bl*K_i_bl);

    % New design
    G_new = new_ctrl.kpx + new_ctrl.kix/s_val + new_ctrl.kdx*s_val/(1+s_val/new_ctrl.omega_px);
    stiff_new_3c(idx) = abs(optimal.total_mass*s_val^2 + K_s_new + G_new*K_i_new);
end

% === PLOT: Part 3c - Dynamic Stiffness ===
figure('Name', 'Part 3c: Dynamic Stiffness', 'Position', [100 100 900 450]);

loglog(freq_range, stiff_bl_3c/1e6, 'b-', 'LineWidth', 2.5); hold on;
loglog(freq_range, stiff_new_3c/1e6, 'r--', 'LineWidth', 2.5);
grid on;
xlabel('Frequency [Hz]', 'FontSize', 12);
ylabel('Dynamic Stiffness [MN/m]', 'FontSize', 12);
title('Part 3c: Dynamic Stiffness Comparison', 'FontSize', 14);
legend('Baseline', 'New Design', 'Location', 'best', 'FontSize', 12);

saveas(gcf, 'part3c_dynamic_stiffness.png');
fprintf('Plot saved: part3c_dynamic_stiffness.png\n');

fprintf('Dynamic Stiffness Comparison:\n');
fprintf('  Frequency    Baseline     New Design\n');
fprintf('  [Hz]         [MN/m]       [MN/m]\n');
fprintf('  ------------------------------------\n');
for f = [10, 100, 500]
    fprintf('  %4.0f         %.2f        %.2f\n', f, ...
        interp1(freq_range, stiff_bl_3c, f)/1e6, ...
        interp1(freq_range, stiff_new_3c, f)/1e6);
end

%% ========================================================================
% DELIVERABLE 3c: ROTOR RUNOUT COMPARISON
% =========================================================================
% Compare rotor runout between baseline and new design.
% =========================================================================

fprintf('\n================================================================\n');
fprintf('PART 3c: Rotor Runout Comparison\n');
fprintf('================================================================\n\n');

SoC_3c = linspace(0, 100, 100);
runout_bl_3c = zeros(size(SoC_3c));
runout_new_3c = zeros(size(SoC_3c));

for idx = 1:length(SoC_3c)
    % Baseline
    w_bl = baseline.omega_min + (baseline.omega_max - baseline.omega_min)*SoC_3c(idx)/100;
    e_bl = (G_grade * 1e-3) / w_bl;
    F_bl = baseline.total_mass * e_bl * w_bl^2;
    s_bl = 1j * w_bl;
    G_bl = baseline.kpx + baseline.kix/s_bl + baseline.kdx*s_bl/(1+s_bl/baseline.omega_px);
    stiff_bl = abs(baseline.total_mass*s_bl^2 + K_s_bl + G_bl*K_i_bl);
    runout_bl_3c(idx) = F_bl / stiff_bl * 1e6;

    % New design
    w_new = omega_min_opt + (omega_max_opt - omega_min_opt)*SoC_3c(idx)/100;
    e_new = (G_grade * 1e-3) / w_new;
    F_new = optimal.total_mass * e_new * w_new^2;
    s_new = 1j * w_new;
    G_new = new_ctrl.kpx + new_ctrl.kix/s_new + new_ctrl.kdx*s_new/(1+s_new/new_ctrl.omega_px);
    stiff_new = abs(optimal.total_mass*s_new^2 + K_s_new + G_new*K_i_new);
    runout_new_3c(idx) = F_new / stiff_new * 1e6;
end

% === PLOT: Part 3c - Rotor Runout ===
figure('Name', 'Part 3c: Rotor Runout', 'Position', [100 100 900 450]);

plot(SoC_3c, runout_bl_3c, 'b-', 'LineWidth', 2.5); hold on;
plot(SoC_3c, runout_new_3c, 'r--', 'LineWidth', 2.5);
grid on;
xlabel('State of Charge [%]', 'FontSize', 12);
ylabel('Rotor Runout [um]', 'FontSize', 12);
title('Part 3c: Rotor Runout Comparison (ISO G2.5)', 'FontSize', 14);
legend('Baseline', 'New Design', 'Location', 'best', 'FontSize', 12);

saveas(gcf, 'part3c_rotor_runout.png');
fprintf('Plot saved: part3c_rotor_runout.png\n');

fprintf('Rotor Runout Comparison:\n');
fprintf('  SoC      Baseline     New Design\n');
fprintf('  [%%]      [um]         [um]\n');
fprintf('  -------------------------------\n');
fprintf('  %3.0f      %.3f        %.3f\n', 0, runout_bl_3c(1), runout_new_3c(1));
fprintf('  %3.0f      %.3f        %.3f\n', 50, interp1(SoC_3c, runout_bl_3c, 50), interp1(SoC_3c, runout_new_3c, 50));
fprintf('  %3.0f      %.3f        %.3f\n', 100, runout_bl_3c(end), runout_new_3c(end));

%% ########################################################################
%                         FINAL SUMMARY
% #########################################################################

fprintf('\n\n');
fprintf('================================================================\n');
fprintf('                     FINAL SUMMARY\n');
fprintf('================================================================\n\n');

fprintf('BASELINE SYSTEM:\n');
fprintf('  Flywheel: %.0f mm dia x %.0f mm length\n', baseline.flywheel_diameter*1000, baseline.flywheel_length*1000);
fprintf('  Mass: %.1f kg\n', baseline.total_mass);
fprintf('  Specific power: %.3f kW/kg\n', specific_power_bl);
fprintf('  Specific energy: %.2f Wh/kg\n', specific_energy_bl);
fprintf('  Cycle efficiency: %.2f%%\n\n', efficiency_1c);

fprintf('OPTIMIZED DESIGN:\n');
fprintf('  Flywheel: %.0f mm dia x %.0f mm length\n', optimal.flywheel_diameter*1000, optimal.flywheel_length*1000);
fprintf('  Mass: %.1f kg\n', optimal.total_mass);
fprintf('  Specific power: %.3f kW/kg (%+.1f%%)\n', optimal.specific_power, ...
    100*(optimal.specific_power - specific_power_bl)/specific_power_bl);
fprintf('  Specific energy: %.2f Wh/kg (%+.1f%%)\n', optimal.specific_energy, ...
    100*(optimal.specific_energy - specific_energy_bl)/specific_energy_bl);
fprintf('  Cycle efficiency: %.2f%%\n\n', optimal.efficiency);

fprintf('AMB CONTROLLER PERFORMANCE:\n');
fprintf('                      Baseline    New Design\n');
fprintf('  Peak displacement:  %.3f um    %.3f um\n', peak_bl, peak_new);
fprintf('  Settling time:      %.2f ms     %.2f ms\n', settle_bl, settle_new);
fprintf('  Max runout:         %.3f um    %.3f um\n\n', max(runout_bl_3c), max(runout_new_3c));

fprintf('================================================================\n');
fprintf('Analysis complete. All plots saved as PNG files.\n');
fprintf('================================================================\n');
