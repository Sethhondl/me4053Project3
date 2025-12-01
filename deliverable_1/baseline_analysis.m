%% DELIVERABLE 1: BASELINE FLYWHEEL SYSTEM ANALYSIS
% Course: Mechanical Engineering Modeling
% Project: Flywheel Energy Storage System Analysis
%
% This script analyzes the baseline flywheel system developed by eXtreme
% Storage Inc. It performs comprehensive characterization including:
%   1a. Losses and temperature vs state of charge
%   1b. Specific power and specific energy calculations
%   1c. Storage cycle efficiency analysis
%   1d. AMB step response analysis
%   1e. Dynamic stiffness frequency response
%   1f. Rotor runout analysis
%
% Required Functions (from EE team):
%   - magneticShear.m
%   - rotorLosses.m
%   - statorLosses.m
%   - ambParameters.m
%   - baselineStorageCycle.m
%
% Author: [Student Name]
% Date: 2025-11-20

clear; close all; clc;

%% Add path to EE functions
% Add path to Project3_Functions folder containing the required functions
addpath('../Project3_Functions');

%% ========================================================================
% SECTION 1: SYSTEM PARAMETERS
% ========================================================================

fprintf('==============================================\n');
fprintf('BASELINE FLYWHEEL SYSTEM ANALYSIS\n');
fprintf('==============================================\n\n');

% Baseline system specifications (from Appendix B - Table A.1)
params = struct();

% Geometric parameters [m]
params.flywheel_length = 1.000;      % Flywheel axial length [m]
params.flywheel_diameter = 0.430;    % Flywheel diameter [m]
params.motor_length = 0.250;         % Motor axial length [m]
params.shaft_diameter = 0.084;       % Shaft and PM diameter [m]
params.magnet_thickness = 0.006;     % Magnet thickness [m]

% Operational parameters
params.max_speed_rpm = 40000;        % Maximum rotational speed [r/min]
params.amb_rated_force = 5780;       % AMB rated force [N]

% Material properties
params.rho_composite = 1600;         % Composite density [kg/m³]
params.rho_steel = 7850;             % Steel density [kg/m³]
params.rho_magnet = 7850;            % Permanent magnet density [kg/m³]

% Safety limits
params.max_steel_tip_speed = 175;    % Max steel tip speed [m/s]
params.max_pm_tip_speed = 175;       % Max PM tip speed [m/s]
params.max_composite_tip_speed = 900; % Max composite tip speed [m/s]
params.max_temp = 100;               % Max safe temperature [°C]

% State of charge definition
% 0% SoC = 50% of max speed, 100% SoC = 100% of max speed
params.min_speed_rpm = params.max_speed_rpm / 2;  % 20,000 r/min at 0% SoC

% Convert to rad/s
params.omega_max = params.max_speed_rpm * 2*pi / 60;  % [rad/s]
params.omega_min = params.min_speed_rpm * 2*pi / 60;  % [rad/s]

% Current controller transfer function: G_ci(s) = 345 + 2149/s (from Appendix B)
params.Kp_current = 345;
params.Ki_current = 2149;

% Housing temperature (from Appendix B)
params.T_housing = 30;  % [°C]

% Emissivities (from Table 1)
params.emissivity_rotor = 0.4;
params.emissivity_housing = 0.9;

% Position controller x (PID with derivative filter) - from Appendix B
params.kpx = 1.2639e8;       % Proportional gain
params.kix = 1.16868e9;      % Integral gain
params.kdx = 252790;         % Derivative gain
params.omega_px = 3770;      % Derivative filter cutoff [rad/s]

% Tilting position controller (PID with derivative filter) - from Appendix B
params.kp_alpha = 7.6992e7;  % Proportional gain
params.ki_alpha = 1.18953e9; % Integral gain
params.kd_alpha = 80294;     % Derivative gain
params.omega_p_alpha = 6283; % Derivative filter cutoff [rad/s]

fprintf('System Parameters Loaded Successfully\n\n');

%% ========================================================================
% SECTION 2: CALCULATE ROTATING GROUP PROPERTIES
% ========================================================================

fprintf('Calculating rotating group mass and inertia...\n');

% Flywheel mass
V_flywheel_outer = pi * (params.flywheel_diameter/2)^2 * params.flywheel_length;
V_flywheel_inner = pi * (params.shaft_diameter/2)^2 * params.flywheel_length;
V_flywheel = V_flywheel_outer - V_flywheel_inner;
m_flywheel = params.rho_composite * V_flywheel;

% Shaft mass (steel cylinder)
% Total shaft length = flywheel + motor + 2*AMBs + clearances
% Estimate shaft length as 1.5 m (includes all components)
shaft_length = 1.5;  % [m] - conservative estimate
V_shaft = pi * (params.shaft_diameter/2)^2 * shaft_length;
m_shaft = params.rho_steel * V_shaft;

% Permanent magnet mass (hollow cylinder on motor section)
magnet_outer_radius = params.shaft_diameter/2 + params.magnet_thickness;
V_magnet_outer = pi * magnet_outer_radius^2 * params.motor_length;
V_magnet_inner = pi * (params.shaft_diameter/2)^2 * params.motor_length;
V_magnet = V_magnet_outer - V_magnet_inner;
m_magnet = params.rho_magnet * V_magnet;

% AMB rotor components (estimate as 10% of shaft mass)
m_amb_rotors = 0.10 * m_shaft;

% Total rotating group mass
m_total = m_flywheel + m_shaft + m_magnet + m_amb_rotors;

% Moment of inertia (treating as hollow cylinder for flywheel + solid for shaft/magnets)
% Flywheel contribution (hollow cylinder)
I_flywheel = 0.5 * m_flywheel * ((params.flywheel_diameter/2)^2 + (params.shaft_diameter/2)^2);

% Shaft contribution (solid cylinder about z-axis)
I_shaft = 0.5 * m_shaft * (params.shaft_diameter/2)^2;

% Magnet contribution (hollow cylinder)
I_magnet = 0.5 * m_magnet * (magnet_outer_radius^2 + (params.shaft_diameter/2)^2);

% Total moment of inertia
I_total = I_flywheel + I_shaft + I_magnet;

fprintf('  Total rotating group mass: %.2f kg\n', m_total);
fprintf('  Total moment of inertia: %.4f kg·m²\n', I_total);
fprintf('  Flywheel mass: %.2f kg\n', m_flywheel);
fprintf('  Shaft mass: %.2f kg\n', m_shaft);
fprintf('  Magnet mass: %.2f kg\n\n', m_magnet);

%% ========================================================================
% PART 1a: LOSSES AND TEMPERATURE VS STATE OF CHARGE
% ========================================================================

fprintf('==============================================\n');
fprintf('PART 1a: Losses and Temperature vs SoC\n');
fprintf('==============================================\n\n');

% Define SoC range
SoC_range = linspace(0, 100, 50);  % 0% to 100% SoC

% Rated current = 1.0 pu (maximum available current)
% Using the .p functions from Project3_Functions to calculate losses
I_rated_pu = 1.0;

fprintf('Using rated current: %.2f pu\n', I_rated_pu);

% Calculate rated power at this current
motor_area = pi * params.shaft_diameter * params.motor_length;
motor_radius = params.shaft_diameter / 2;
shear_rated = magneticShear(params.magnet_thickness, I_rated_pu);
torque_rated = shear_rated * motor_area * motor_radius;
power_rated = torque_rated * params.omega_max;  % [W]

fprintf('Calculated rated power: %.2f kW\n', power_rated/1000);
fprintf('Analyzing losses at rated power across SoC range...\n\n');

% Initialize arrays
omega_array = zeros(size(SoC_range));
rotor_loss_array = zeros(size(SoC_range));
stator_loss_array = zeros(size(SoC_range));
total_loss_array = zeros(size(SoC_range));
temperature_array = zeros(size(SoC_range));
current_array = zeros(size(SoC_range));

% Loop through SoC values
for i = 1:length(SoC_range)
    % Calculate speed for this SoC
    omega = params.omega_min + (params.omega_max - params.omega_min) * SoC_range(i)/100;
    omega_array(i) = omega;

    % Calculate current needed to produce rated power at this speed
    % Power = shear * A * r * omega
    % Current scales to maintain constant power as speed changes
    I_stator_pu = I_rated_pu * (params.omega_max / omega);
    I_stator_pu = min(I_stator_pu, 1.0);  % Limit to maximum (1.0 pu)
    current_array(i) = I_stator_pu;

    % Calculate losses
    P_rotor = rotorLosses(params.magnet_thickness, params.shaft_diameter, ...
        params.motor_length, I_stator_pu, omega*60/(2*pi));  % Convert to r/min
    P_stator = statorLosses(params.magnet_thickness, params.shaft_diameter, ...
        params.motor_length, I_stator_pu, omega*60/(2*pi));  % Convert to r/min

    rotor_loss_array(i) = P_rotor;
    stator_loss_array(i) = P_stator;
    total_loss_array(i) = P_rotor + P_stator;

    % Calculate temperature rise using radiation heat transfer
    % In vacuum, only radiation between rotor and housing
    % Two-surface enclosure: Q = σ * A_rotor * (T_rotor^4 - T_housing^4) /
    %                            (1/ε_rotor + (A_rotor/A_housing)*(1/ε_housing - 1))
    % NOTE: Only ROTOR losses heat the rotor (stator is stationary, outside vacuum)
    sigma = 5.67e-8;  % Stefan-Boltzmann constant [W/(m²·K⁴)]
    T_housing = params.T_housing + 273;  % Housing temperature [K] = 30°C

    % Rotor surface area (flywheel cylinder)
    A_rotor = 2*pi*(params.flywheel_diameter/2)*params.flywheel_length + ...
        2*pi*(params.flywheel_diameter/2)^2;  % [m²]

    % Housing surface area (with 20mm radial clearance from flywheel)
    housing_inner_diameter = params.flywheel_diameter + 2*0.020;  % 20mm clearance
    A_housing = 2*pi*(housing_inner_diameter/2)*params.flywheel_length + ...
        2*pi*(housing_inner_diameter/2)^2;  % [m²]

    % Radiation resistance factor for two-surface enclosure
    rad_factor = 1/params.emissivity_rotor + ...
        (A_rotor/A_housing)*(1/params.emissivity_housing - 1);

    % Solve for rotor temperature: Q = σ * A_rotor * (T_rotor^4 - T_housing^4) / rad_factor
    % T_rotor^4 = T_housing^4 + Q * rad_factor / (σ * A_rotor)
    T_rotor = (T_housing^4 + rotor_loss_array(i)*rad_factor/(sigma*A_rotor))^(1/4);
    temperature_array(i) = T_rotor - 273;  % Convert to Celsius
end

% Create figure for Part 1a
figure('Name', 'Part 1a: Losses and Temperature vs SoC', 'Position', [100 100 1000 500]);

% Plot losses
subplot(1,2,1);
plot(SoC_range, rotor_loss_array/1000, 'r-', 'LineWidth', 2); hold on;
plot(SoC_range, stator_loss_array/1000, 'b-', 'LineWidth', 2);
plot(SoC_range, total_loss_array/1000, 'k-', 'LineWidth', 2.5);
grid on;
xlabel('State of Charge [%]', 'FontSize', 12);
ylabel('Power Loss [kW]', 'FontSize', 12);
title('Losses at Rated Power vs SoC', 'FontSize', 14);
legend('Rotor Losses', 'Stator Losses', 'Total Losses', 'Location', 'best');

% Plot temperature
subplot(1,2,2);
plot(SoC_range, temperature_array, 'k-', 'LineWidth', 2.5); hold on;
yline(params.max_temp, 'r--', 'LineWidth', 2, 'Label', 'Max Safe Temp');
grid on;
xlabel('State of Charge [%]', 'FontSize', 12);
ylabel('Rotating Group Temperature [°C]', 'FontSize', 12);
title('Temperature vs SoC', 'FontSize', 14);

% Save figure
saveas(gcf, 'part1a_losses_temperature.fig');
saveas(gcf, 'part1a_losses_temperature.png');

fprintf('Part 1a completed. Plots saved.\n');
fprintf('  Max total losses: %.2f kW at %.0f%% SoC\n', max(total_loss_array)/1000, SoC_range(find(total_loss_array==max(total_loss_array),1)));
fprintf('  Max temperature: %.2f °C at %.0f%% SoC\n\n', max(temperature_array), SoC_range(find(temperature_array==max(temperature_array),1)));

%% ========================================================================
% PART 1b: SPECIFIC POWER AND SPECIFIC ENERGY
% ========================================================================

fprintf('==============================================\n');
fprintf('PART 1b: Specific Power and Specific Energy\n');
fprintf('==============================================\n\n');

% Specific power [kW/kg]
% Using rated power calculated above
specific_power = (power_rated / 1000) / m_total;  % [kW/kg]

% Specific energy [Wh/kg]
% Energy stored between 0% and 100% SoC
E_max = 0.5 * I_total * params.omega_max^2;  % [J]
E_min = 0.5 * I_total * params.omega_min^2;  % [J]
E_stored = E_max - E_min;  % [J]
E_stored_Wh = E_stored / 3600;  % Convert to [Wh]
specific_energy = E_stored_Wh / m_total;  % [Wh/kg]

fprintf('Specific Power: %.3f kW/kg\n', specific_power);
fprintf('Specific Energy: %.3f Wh/kg\n', specific_energy);
fprintf('Total Energy Storage: %.2f kWh\n\n', E_stored_Wh/1000);

%% ========================================================================
% PART 1c: STORAGE CYCLE EFFICIENCY
% ========================================================================

fprintf('==============================================\n');
fprintf('PART 1c: Storage Cycle Efficiency\n');
fprintf('==============================================\n\n');

% Simulate storage cycle starting at 50% SoC
dt = 1.0;  % Time step [s]
t_cycle = 0:dt:900;  % 15-minute cycle [s]
SoC_initial = 50;  % Start at 50% SoC

% Initialize state
omega_current = params.omega_min + (params.omega_max - params.omega_min) * SoC_initial/100;
E_current = 0.5 * I_total * omega_current^2;

% Arrays for tracking
omega_cycle = zeros(size(t_cycle));
E_cycle = zeros(size(t_cycle));
power_grid = zeros(size(t_cycle));
power_losses = zeros(size(t_cycle));
SoC_cycle = zeros(size(t_cycle));

% Energy accounting
E_in = 0;   % Energy from grid (charging)
E_out = 0;  % Energy to grid (discharging)

fprintf('Simulating 15-minute storage cycle...\n');

for i = 1:length(t_cycle)
    % Get grid power demand (positive = discharge to grid)
    P_grid = baselineStorageCycle(t_cycle(i));
    power_grid(i) = P_grid;

    % Calculate current speed and SoC
    omega_rpm = omega_current * 60 / (2*pi);
    SoC_current = 100 * (omega_current - params.omega_min) / (params.omega_max - params.omega_min);

    % Calculate losses at current operating point
    % Scale current based on power demand relative to rated power
    I_stator_current_pu = I_rated_pu * abs(P_grid) / power_rated;
    I_stator_current_pu = min(I_stator_current_pu, 1.0);  % Limit to max (1.0 pu)

    P_rotor_loss = rotorLosses(params.magnet_thickness, params.shaft_diameter, ...
        params.motor_length, I_stator_current_pu, omega_rpm);
    P_stator_loss = statorLosses(params.magnet_thickness, params.shaft_diameter, ...
        params.motor_length, I_stator_current_pu, omega_rpm);
    P_loss_total = P_rotor_loss + P_stator_loss;
    power_losses(i) = P_loss_total;

    % Net power to/from rotor (negative = charging rotor)
    P_net = P_grid + P_loss_total;  % Grid power + losses

    % Update energy and speed
    dE = -P_net * dt;  % Energy change (negative P_net = energy in)
    E_current = E_current + dE;

    % Calculate new angular velocity from energy
    omega_current = sqrt(2 * E_current / I_total);

    % Limit speed to valid range
    omega_current = max(params.omega_min, min(params.omega_max, omega_current));

    % Store values
    omega_cycle(i) = omega_current;
    E_cycle(i) = E_current;
    SoC_cycle(i) = 100 * (omega_current - params.omega_min) / (params.omega_max - params.omega_min);

    % Track energy in/out
    if P_grid > 0  % Discharging to grid
        E_out = E_out + P_grid * dt;
    else  % Charging from grid
        E_in = E_in + abs(P_grid) * dt;
    end
end

% Calculate total self-discharge loss
E_loss_total = sum(power_losses) * dt;

% Calculate efficiency
% Efficiency = E_out / (E_in + E_recovery)
% where E_recovery is energy needed to restore initial SoC
E_recovery = 0.5 * I_total * (omega_cycle(1)^2 - omega_cycle(end)^2);
if E_recovery < 0
    E_recovery = 0;  % No recovery needed if SoC increased
end

efficiency = (E_out / (E_in + E_recovery)) * 100;  % [%]

% Create figure for Part 1c
figure('Name', 'Part 1c: Storage Cycle Analysis', 'Position', [100 100 1000 800]);

subplot(3,1,1);
plot(t_cycle/60, power_grid/1000, 'b-', 'LineWidth', 2);
grid on;
xlabel('Time [min]', 'FontSize', 12);
ylabel('Grid Power [kW]', 'FontSize', 12);
title('Grid Power Demand (Baseline Cycle)', 'FontSize', 14);

subplot(3,1,2);
plot(t_cycle/60, SoC_cycle, 'r-', 'LineWidth', 2);
grid on;
xlabel('Time [min]', 'FontSize', 12);
ylabel('State of Charge [%]', 'FontSize', 12);
title('Flywheel SoC During Cycle', 'FontSize', 14);

subplot(3,1,3);
plot(t_cycle/60, power_losses/1000, 'k-', 'LineWidth', 2);
grid on;
xlabel('Time [min]', 'FontSize', 12);
ylabel('Total Losses [kW]', 'FontSize', 12);
title('Rotor and Stator Losses', 'FontSize', 14);

% Save figure
saveas(gcf, 'part1c_storage_cycle.fig');
saveas(gcf, 'part1c_storage_cycle.png');

fprintf('Part 1c completed. Plots saved.\n');
fprintf('  Energy discharged to grid: %.2f kWh\n', E_out/3.6e6);
fprintf('  Energy charged from grid: %.2f kWh\n', E_in/3.6e6);
fprintf('  Self-discharge losses: %.2f kWh\n', E_loss_total/3.6e6);
fprintf('  Recovery energy needed: %.2f kWh\n', E_recovery/3.6e6);
fprintf('  Cycle efficiency: %.2f%%\n\n', efficiency);

%% ========================================================================
% PART 1d: AMB STEP RESPONSE
% ========================================================================

fprintf('==============================================\n');
fprintf('PART 1d: AMB Step Response Analysis\n');
fprintf('==============================================\n\n');

% Get AMB parameters
amb_params = ambParameters(params.shaft_diameter, params.amb_rated_force);

% Extract parameters (note: stiffnessConstant is positive, but represents negative stiffness)
K_s = -amb_params.stiffnessConstant;  % Position stiffness (negative, destabilizing) [N/m]
K_i = amb_params.forceConstant;        % Current-to-force constant [N/A]
amb_mass = m_total;                    % Use calculated rotating group mass [kg]

fprintf('AMB Parameters:\n');
fprintf('  Position stiffness (K_s): %.2f N/m (negative/destabilizing)\n', K_s);
fprintf('  Force constant (K_i): %.2f N/A\n', K_i);
fprintf('  Bias current: %.3f A\n', amb_params.biasCurrent);
fprintf('  Rated control current: %.3f A\n', amb_params.ratedControlCurrent);
fprintf('  Coil inductance: %.4f H\n', amb_params.coilInductance);
fprintf('  Coil resistance: %.3f Ohms\n', amb_params.coilResistance);
fprintf('  Axial length: %.4f m\n', amb_params.axialLength);

% Calculate unstable pole frequency (HW3 Q2d methodology)
% Plant: Y(s)/I(s) = Ki / (m*s^2 - |Ks|) has poles at s = ±sqrt(|Ks|/m)
f_unstable = sqrt(abs(K_s) / amb_mass) / (2*pi);
fprintf('  Unstable pole frequency: %.2f Hz\n', f_unstable);

% Use position controller from Appendix B (PID with derivative filter)
% Transfer function: G(s) = kpx + kix/s + s*kdx/(1 + s/omega_px)
fprintf('\nPosition Controller (from Appendix B):\n');
fprintf('  Kp = %.4e\n', params.kpx);
fprintf('  Ki = %.4e\n', params.kix);
fprintf('  Kd = %.4e\n', params.kdx);
fprintf('  omega_p = %.1f rad/s\n\n', params.omega_px);

% Step disturbance: 10% of top AMB's rated force
F_step = 0.10 * params.amb_rated_force;  % [N]

fprintf('Simulating AMB step response to %.1f N disturbance at zero speed...\n', F_step);
fprintf('Using transfer function approach for accurate high-gain simulation...\n');

% Build transfer functions for proper closed-loop analysis
s = tf('s');

% Position controller: G_cx(s) = kpx + kix/s + s*kdx/(1 + s/omega_px)
G_cx = params.kpx + params.kix/s + s*params.kdx/(1 + s/params.omega_px);

% Current controller: G_ci(s) = Kp + Ki/s (simplified as fast first-order for analysis)
% For step response, approximate as unity gain (current loop much faster than position loop)
G_ci = 1;  % Fast current loop approximation

% AMB plant: Force to displacement
% m*x'' = F_amb + F_dist = K_i*i + K_s*x + F_dist
% Plant from control current to position: X(s)/I(s) = K_i / (m*s^2 - K_s)
% Note: K_s < 0, so denominator is m*s^2 + |K_s|
G_plant = K_i / (amb_mass*s^2 + K_s);

% Open-loop transfer function: position error to position
L = G_cx * G_ci * G_plant;

% Closed-loop transfer function from disturbance to position
% X(s)/F_dist(s) = G_dist / (1 + L)
% where G_dist = 1/(m*s^2 - K_s)
G_dist = 1 / (amb_mass*s^2 + K_s);
T_dist = G_dist / (1 + L);

% Closed-loop transfer function from disturbance to control force
% F_ctrl(s)/F_dist(s) = -L / (1 + L) * G_dist * (m*s^2 - K_s) / K_i
T_force = -L * G_dist / (1 + L);

% Simulate step response
t_amb = linspace(0, 0.05, 5000);  % 50 ms simulation

% Position response to step disturbance
[x_array, t_out] = step(T_dist * F_step, t_amb);

% Force response (AMB force countering disturbance)
[f_ctrl, ~] = step(T_force * F_step * K_i, t_amb);

% Current response (force / K_i)
i_array = f_ctrl / K_i;

% Calculate total AMB force (should approach -F_step at steady state)
f_array = f_ctrl;

% Create figure for Part 1d
figure('Name', 'Part 1d: AMB Step Response', 'Position', [100 100 1000 800]);

% Current
subplot(3,1,1);
plot(t_out*1000, i_array, 'b-', 'LineWidth', 2);
grid on;
xlabel('Time [ms]', 'FontSize', 11);
ylabel('Current [A]', 'FontSize', 11);
title('AMB Current Response - Zero Speed', 'FontSize', 12);

% Force
subplot(3,1,2);
plot(t_out*1000, f_array, 'r-', 'LineWidth', 2);
hold on;
yline(-F_step, 'k--', 'LineWidth', 1.5, 'Label', 'Target (-F_{dist})');
grid on;
xlabel('Time [ms]', 'FontSize', 11);
ylabel('AMB Force [N]', 'FontSize', 11);
title('AMB Force Response - Zero Speed', 'FontSize', 12);

% Rotor position
subplot(3,1,3);
plot(t_out*1000, x_array*1e6, 'k-', 'LineWidth', 2);
grid on;
xlabel('Time [ms]', 'FontSize', 11);
ylabel('Rotor Position [\mum]', 'FontSize', 11);
title('Rotor Position Response - Zero Speed', 'FontSize', 12);

% Save figure
saveas(gcf, 'part1d_amb_step_response.fig');
saveas(gcf, 'part1d_amb_step_response.png');

fprintf('Part 1d completed. Plots saved.\n');
fprintf('  Max displacement: %.4f µm\n', max(abs(x_array))*1e6);
fprintf('  Final displacement: %.4f µm\n', abs(x_array(end))*1e6);

% Check stability by examining if response settles
if abs(x_array(end)) < 0.1 * max(abs(x_array)) || max(abs(x_array)) < 1e-6
    fprintf('  System is stable (response settles)\n\n');
else
    fprintf('  WARNING: System response may not be settling properly\n\n');
end

%% ========================================================================
% PART 1e: DYNAMIC STIFFNESS
% ========================================================================

fprintf('==============================================\n');
fprintf('PART 1e: Dynamic Stiffness Analysis\n');
fprintf('==============================================\n\n');

% Frequency range for analysis
freq_range = logspace(0, 3, 200);  % 1 Hz to 1000 Hz
omega_range = 2*pi*freq_range;  % Convert to rad/s

% Position controller transfer function (from Appendix B):
% G_cx(s) = kpx + kix/s + s*kdx/(1 + s/omega_px)
%
% For the closed-loop system with AMB:
% Plant: G_p(s) = K_i / (m*s^2 - K_s)  where K_s < 0 (destabilizing)
%
% Dynamic stiffness = F_dist / X = 1 / (closed-loop X/F_dist)

stiffness_radial = zeros(size(omega_range));
stiffness_tilt = zeros(size(omega_range));

% Radial stiffness using specified position controller
fprintf('Computing radial dynamic stiffness...\n');
for idx = 1:length(omega_range)
    s = 1j * omega_range(idx);

    % Position controller transfer function (PID with derivative filter)
    % G_cx(s) = kpx + kix/s + s*kdx/(1 + s/omega_px)
    G_cx = params.kpx + params.kix/s + s*params.kdx/(1 + s/params.omega_px);

    % Open-loop: L(s) = G_cx(s) * K_i / (m*s^2 - K_s)
    % Note: K_s is already negative (destabilizing stiffness)
    plant_num = K_i;
    plant_den = amb_mass*s^2 + K_s;  % K_s < 0, so this is m*s^2 - |K_s|

    L = G_cx * plant_num / plant_den;

    % Closed-loop: X/F_dist = (1/plant_den) / (1 + L)
    %            = 1 / (plant_den + G_cx * K_i)
    % Dynamic stiffness = 1 / (X/F_dist) = plant_den + G_cx * K_i
    dyn_stiff = plant_den + G_cx * K_i;
    stiffness_radial(idx) = abs(dyn_stiff);
end

% Tilting stiffness using tilting controller (from Appendix B)
% G_alpha(s) = kp_alpha + ki_alpha/s + s*kd_alpha/(1 + s/omega_p_alpha)
fprintf('Computing tilting dynamic stiffness...\n');

% AMB separation distance (estimated from geometry)
L_amb = params.flywheel_length + 0.3;  % ~1.3 m separation

% Transverse moment of inertia (for tilting motion)
% Approximate as solid cylinder for simplicity
I_transverse = (1/12) * m_total * (3*(params.flywheel_diameter/2)^2 + params.flywheel_length^2);

for idx = 1:length(omega_range)
    s = 1j * omega_range(idx);

    % Tilting controller transfer function
    G_alpha = params.kp_alpha + params.ki_alpha/s + s*params.kd_alpha/(1 + s/params.omega_p_alpha);

    % For tilting mode, the effective stiffness relates torque to angle
    % Each AMB contributes force at distance L_amb/2 from center
    % Effective moment arm = L_amb/2
    % The plant for tilting: theta/M = 1/(I_transverse*s^2 - K_s_tilt)
    % where K_s_tilt ≈ K_s * (L_amb/2)^2 * 2 for two AMBs

    K_s_tilt = K_s * (L_amb/2)^2 * 2;  % Tilting stiffness (negative)
    K_i_tilt = K_i * (L_amb/2) * 2;    % Tilting force constant

    plant_den_tilt = I_transverse*s^2 + K_s_tilt;

    % Dynamic stiffness for tilting (torque per radian)
    dyn_stiff_tilt = plant_den_tilt + G_alpha * K_i_tilt;

    % Convert to equivalent linear stiffness at AMB location
    % F/x = M/theta * (1/r^2) where r = L_amb/2
    stiffness_tilt(idx) = abs(dyn_stiff_tilt) / (L_amb/2)^2;
end

% Create figure for Part 1e
figure('Name', 'Part 1e: Dynamic Stiffness', 'Position', [100 100 900 500]);

loglog(freq_range, stiffness_radial/1e6, 'b-', 'LineWidth', 2.5); hold on;
loglog(freq_range, stiffness_tilt/1e6, 'r-', 'LineWidth', 2.5);
grid on;
xlabel('Frequency [Hz]', 'FontSize', 12);
ylabel('Dynamic Stiffness [MN/m]', 'FontSize', 12);
title('AMB Dynamic Stiffness vs Frequency', 'FontSize', 14);
legend('Radial Direction', 'Tilting Direction', 'Location', 'best', 'FontSize', 12);

% Save figure
saveas(gcf, 'part1e_dynamic_stiffness.fig');
saveas(gcf, 'part1e_dynamic_stiffness.png');

fprintf('Part 1e completed. Plots saved.\n');
fprintf('  Radial stiffness at 10 Hz: %.2f MN/m\n', ...
    interp1(freq_range, stiffness_radial, 10)/1e6);
fprintf('  Radial stiffness at 100 Hz: %.2f MN/m\n', ...
    interp1(freq_range, stiffness_radial, 100)/1e6);
fprintf('  Tilting stiffness at 10 Hz: %.2f MN/m\n', ...
    interp1(freq_range, stiffness_tilt, 10)/1e6);
fprintf('  Tilting stiffness at 100 Hz: %.2f MN/m\n\n', ...
    interp1(freq_range, stiffness_tilt, 100)/1e6);

%% ========================================================================
% PART 1f: ROTOR RUNOUT
% ========================================================================

fprintf('==============================================\n');
fprintf('PART 1f: Rotor Runout Analysis\n');
fprintf('==============================================\n\n');

% Mass imbalance assumption: ISO G2.5 balance grade (from Table 1)
% G2.5 means: e*omega = 2.5 mm/s (where e = eccentricity, omega = angular velocity)
% This is typical for precision rotors

G_grade = 2.5;  % ISO balance grade G2.5 [mm/s]

% Calculate runout for each SoC using the specified controller
SoC_runout = linspace(0, 100, 100);
runout_array = zeros(size(SoC_runout));
omega_runout_array = zeros(size(SoC_runout));

fprintf('Computing rotor runout using specified PID controller...\n');

for idx = 1:length(SoC_runout)
    % Speed at this SoC
    omega = params.omega_min + (params.omega_max - params.omega_min) * SoC_runout(idx)/100;
    omega_runout_array(idx) = omega * 60 / (2*pi);  % Convert to RPM

    % Eccentricity from balance grade
    e = (G_grade * 1e-3) / omega;  % [m] - eccentricity

    % Unbalance force amplitude (synchronous disturbance at rotation frequency)
    F_unbalance = m_total * e * omega^2;

    % Calculate closed-loop response at synchronous frequency
    s = 1j * omega;  % Excitation at rotation frequency

    % Position controller transfer function (PID with derivative filter)
    G_cx = params.kpx + params.kix/s + s*params.kdx/(1 + s/params.omega_px);

    % Plant: mass with negative stiffness
    plant_den = amb_mass*s^2 + K_s;  % K_s < 0 (destabilizing)

    % Dynamic stiffness at this frequency
    dyn_stiff = plant_den + G_cx * K_i;

    % Runout = Force / Dynamic_Stiffness
    runout = F_unbalance / abs(dyn_stiff);

    runout_array(idx) = runout * 1e6;  % Convert to micrometers
end

% Create figure for Part 1f
figure('Name', 'Part 1f: Rotor Runout', 'Position', [100 100 900 500]);

plot(SoC_runout, runout_array, 'b-', 'LineWidth', 2.5);
grid on;
xlabel('State of Charge [%]', 'FontSize', 12);
ylabel('Rotor Runout [\mum]', 'FontSize', 12);
title('Rotor Runout Due to Mass Imbalance (ISO G2.5) vs SoC', 'FontSize', 14);

% Add secondary x-axis for RPM
ax1 = gca;
ax2 = axes('Position', ax1.Position, 'XAxisLocation', 'top', 'Color', 'none');
ax2.XLim = [params.min_speed_rpm params.max_speed_rpm]/1000;
ax2.YTick = [];
xlabel(ax2, 'Rotational Speed [krpm]', 'FontSize', 12);

% Save figure
saveas(gcf, 'part1f_rotor_runout.fig');
saveas(gcf, 'part1f_rotor_runout.png');

fprintf('Part 1f completed. Plots saved.\n');
fprintf('  Runout at 0%% SoC (20k RPM): %.4f µm\n', runout_array(1));
fprintf('  Runout at 50%% SoC (30k RPM): %.4f µm\n', interp1(SoC_runout, runout_array, 50));
fprintf('  Runout at 100%% SoC (40k RPM): %.4f µm\n\n', runout_array(end));

%% ========================================================================
% SUMMARY
% ========================================================================

fprintf('==============================================\n');
fprintf('DELIVERABLE 1 ANALYSIS COMPLETE\n');
fprintf('==============================================\n\n');

fprintf('KEY RESULTS SUMMARY:\n\n');

fprintf('1a. Losses and Temperature:\n');
fprintf('    - Max losses: %.2f kW\n', max(total_loss_array)/1000);
fprintf('    - Max temperature: %.2f °C\n\n', max(temperature_array));

fprintf('1b. Specific Performance:\n');
fprintf('    - Specific power: %.3f kW/kg\n', specific_power);
fprintf('    - Specific energy: %.3f Wh/kg\n\n', specific_energy);

fprintf('1c. Storage Cycle:\n');
fprintf('    - Cycle efficiency: %.2f%%\n\n', efficiency);

fprintf('1d. AMB Step Response:\n');
fprintf('    - Using PID controller from Appendix B\n');
fprintf('    - Kp=%.4e, Ki=%.4e, Kd=%.4e\n', params.kpx, params.kix, params.kdx);
fprintf('    - System response at zero speed\n\n');

fprintf('1e. Dynamic Stiffness:\n');
fprintf('    - Frequency response analyzed from 1-1000 Hz\n');
fprintf('    - Using specified radial and tilting controllers\n\n');

fprintf('1f. Rotor Runout:\n');
fprintf('    - ISO G2.5 balance grade (from Table 1)\n');
fprintf('    - Max runout: %.4f µm\n\n', max(runout_array));

fprintf('All figures saved to current folder\n');
fprintf('==============================================\n');
