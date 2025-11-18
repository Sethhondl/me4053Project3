% analyze_baseline.m - Baseline flywheel system analysis
%
% This script performs all required analyses for the baseline system:
% 1a. Plot losses and temperature vs SoC at rated power
% 1b. Calculate specific power and specific energy
% 1c. Calculate efficiency for storage cycle
% 1d. Plot AMB step response
% 1e. Plot dynamic stiffness vs frequency
% 1f. Plot rotor runout vs SoC

clear; clc; close all;

% Add paths to utility functions and EE functions
addpath('../utilities');
addpath('../ee_functions');

fprintf('=== BASELINE FLYWHEEL SYSTEM ANALYSIS ===\n\n');

%% Load baseline configuration
config = baseline_config();

%% Extract key parameters
omega_max = config.omega_max;
P_rated = config.P_rated;
I_total = config.I_total;
mass_total = config.mass_total;
dims = config.dims;
magnet_thickness = config.magnet_thickness;
T_ambient = config.T_ambient;

%% 1a. Losses and Temperature vs SoC at Rated Power

fprintf('1a. Calculating losses and temperature vs SoC...\n');

% SoC array
SoC_array = linspace(0, 1, 50);
n_points = length(SoC_array);

% Preallocate arrays
omega_array = zeros(n_points, 1);
P_rotor_array = zeros(n_points, 1);
P_stator_array = zeros(n_points, 1);
P_total_array = zeros(n_points, 1);
T_rotor_array = zeros(n_points, 1);
T_stator_array = zeros(n_points, 1);
I_stator_array = zeros(n_points, 1);

for i = 1:n_points
    % Convert SoC to speed
    omega = soc2speed(SoC_array(i), omega_max);
    omega_array(i) = omega;

    % Calculate torque required for rated power
    % P = T * omega  =>  T = P / omega
    if omega > 0
        T_rated = P_rated / omega;
    else
        T_rated = 0;
    end

    % Calculate stator current required for rated power
    % T = k_i * r * I  where k_i is from magnetic shear stress
    % tau = F/A = T/(r*A) where A = 2*pi*r*l
    % tau = k_i * I  (from magneticShear function)
    % So: T = tau * 2*pi*r^2*l = k_i * I * 2*pi*r^2*l

    r_rotor = dims.r_pm_outer;
    l_motor = dims.l_motor;
    A_rotor = 2 * pi * r_rotor * l_motor;

    % Iterative solution for stator current (simplified)
    I_stator = 50; % Initial guess [A]
    for iter = 1:10
        tau = magneticShear(magnet_thickness, I_stator);
        T_available = tau * A_rotor * r_rotor;
        if T_available > 0
            I_stator = I_stator * T_rated / T_available;
        end
    end
    I_stator_array(i) = I_stator;

    % Calculate losses
    P_rotor = rotorLosses(magnet_thickness, 2*r_rotor, l_motor, I_stator, omega);
    P_stator = statorLosses(magnet_thickness, 2*r_rotor, l_motor, I_stator, omega);

    P_rotor_array(i) = P_rotor;
    P_stator_array(i) = P_stator;
    P_total_array(i) = P_rotor + P_stator;

    % Calculate temperatures
    [T_rotor, T_stator] = calculateRotorTemperature(P_rotor, P_stator, dims, omega, T_ambient);
    T_rotor_array(i) = T_rotor;
    T_stator_array(i) = T_stator;
end

% Plot results
figure('Name', 'Baseline: Losses and Temperature vs SoC');
subplot(2,1,1);
plot(SoC_array*100, P_rotor_array/1000, 'r-', 'LineWidth', 2); hold on;
plot(SoC_array*100, P_stator_array/1000, 'b-', 'LineWidth', 2);
plot(SoC_array*100, P_total_array/1000, 'k--', 'LineWidth', 2);
grid on;
xlabel('State of Charge [%]');
ylabel('Power Loss [kW]');
legend('Rotor Losses', 'Stator Losses', 'Total Losses', 'Location', 'best');
title('Losses vs State of Charge at Rated Power');

subplot(2,1,2);
plot(SoC_array*100, T_rotor_array, 'r-', 'LineWidth', 2); hold on;
plot(SoC_array*100, T_stator_array, 'b-', 'LineWidth', 2);
yline(100, 'k--', 'Max Safe Temp', 'LineWidth', 1.5);
grid on;
xlabel('State of Charge [%]');
ylabel('Temperature [°C]');
legend('Rotor Temp', 'Stator Temp', 'Location', 'best');
title('Temperature vs State of Charge at Rated Power');

saveas(gcf, '../results/baseline_losses_temp_vs_soc.fig');
saveas(gcf, '../results/baseline_losses_temp_vs_soc.png');

%% 1b. Specific Power and Specific Energy

fprintf('1b. Calculating specific power and specific energy...\n');

% Specific power [kW/kg] - maximum power divided by mass
specific_power = P_rated / mass_total / 1000; % kW/kg

% Specific energy [kWh/kg] - usable energy divided by mass
[~, E_usable] = flywheelEnergy(omega_max, I_total);
specific_energy = E_usable / mass_total / 3.6e6; % kWh/kg

fprintf('  Specific Power: %.3f kW/kg\n', specific_power);
fprintf('  Specific Energy: %.4f kWh/kg\n', specific_energy);

%% 1c. Storage Cycle Efficiency

fprintf('1c. Calculating storage cycle efficiency...\n');
fprintf('  NOTE: Requires actual storage cycle data (power vs time profile)\n');
fprintf('  Using placeholder analysis for now\n\n');

% Placeholder storage cycle (replace with actual cycle from Canvas)
% Assume a simple charge-discharge cycle
t_cycle = 0:60:3600; % 1 hour cycle, 1 minute resolution [s]
P_cycle = zeros(size(t_cycle));
% Example: 15 min charge, 30 min standby, 15 min discharge
P_cycle(1:15) = P_rated;           % Charge
P_cycle(16:45) = 0;                 % Standby
P_cycle(46:60) = -P_rated;          % Discharge

% Simulate storage cycle starting at 50% SoC
efficiency_data = simulateStorageCycle(config, t_cycle, P_cycle, 0.5);

fprintf('  Cycle Efficiency: %.2f%%\n', efficiency_data.efficiency * 100);
fprintf('  Energy in: %.2f kWh\n', efficiency_data.E_in / 3.6e6);
fprintf('  Energy out: %.2f kWh\n', efficiency_data.E_out / 3.6e6);
fprintf('  Self-discharge losses: %.2f kWh\n', efficiency_data.E_selfdischarge / 3.6e6);

% Plot storage cycle
figure('Name', 'Baseline: Storage Cycle');
subplot(3,1,1);
plot(t_cycle/60, P_cycle/1000, 'k-', 'LineWidth', 2);
grid on;
xlabel('Time [min]');
ylabel('Power [kW]');
title('Storage Cycle Power Profile');

subplot(3,1,2);
plot(efficiency_data.t_sim/60, efficiency_data.SoC_sim*100, 'b-', 'LineWidth', 2);
grid on;
xlabel('Time [min]');
ylabel('State of Charge [%]');
title('SoC During Storage Cycle');

subplot(3,1,3);
plot(efficiency_data.t_sim/60, efficiency_data.T_rotor_sim, 'r-', 'LineWidth', 2);
hold on;
yline(100, 'k--', 'Max Safe Temp', 'LineWidth', 1.5);
grid on;
xlabel('Time [min]');
ylabel('Rotor Temperature [°C]');
title('Rotor Temperature During Cycle');

saveas(gcf, '../results/baseline_storage_cycle.fig');
saveas(gcf, '../results/baseline_storage_cycle.png');

%% Save baseline results
baseline_results = struct();
baseline_results.config = config;
baseline_results.SoC_array = SoC_array;
baseline_results.P_rotor_array = P_rotor_array;
baseline_results.P_stator_array = P_stator_array;
baseline_results.T_rotor_array = T_rotor_array;
baseline_results.specific_power = specific_power;
baseline_results.specific_energy = specific_energy;
baseline_results.efficiency_data = efficiency_data;

save('../results/baseline_results.mat', 'baseline_results');

fprintf('\n=== BASELINE ANALYSIS COMPLETE ===\n');
fprintf('Results saved to ../results/\n\n');

fprintf('NOTE: Still need to implement:\n');
fprintf('  1d. AMB step response\n');
fprintf('  1e. Dynamic stiffness vs frequency\n');
fprintf('  1f. Rotor runout vs SoC\n');
fprintf('These require AMB controller design (see AMB scripts)\n\n');
