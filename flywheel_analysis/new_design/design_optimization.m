% design_optimization.m - Design space exploration for new flywheel system
%
% This script performs design optimization to find optimal flywheel parameters:
% Design variables:
%   - Maximum rotational speed (omega_max)
%   - Magnet thickness
%
% Objectives:
%   - Minimize losses
%   - Maximize specific power
%   - Maximize specific energy
%
% Constraints:
%   - Satisfy new storage cycle power requirements
%   - Start at 50% SoC, maintain SoC > 0% throughout cycle
%   - Maximum rotating group temperature < 100°C
%   - Safety limits on tip speeds

clear; clc; close all;

% Add paths
addpath('../utilities');
addpath('../ee_functions');
addpath('../baseline_analysis');

fprintf('=== NEW FLYWHEEL DESIGN OPTIMIZATION ===\n\n');

%% Load new storage cycle
% PLACEHOLDER - Replace with actual storage cycle from Canvas
fprintf('Loading new storage cycle...\n');
fprintf('  NOTE: Using placeholder cycle. Replace with actual from Canvas!\n\n');

% Placeholder cycle (replace with actual)
t_new_cycle = 0:60:7200; % 2 hours, 1 minute resolution [s]
P_new_cycle = zeros(size(t_new_cycle));

% Example: Variable power profile
% First hour: alternating charge/discharge
P_new_cycle(1:15) = 150e3;      % 15 min charge at 150 kW
P_new_cycle(16:30) = -100e3;    % 15 min discharge at 100 kW
P_new_cycle(31:45) = 200e3;     % 15 min charge at 200 kW
P_new_cycle(46:60) = -150e3;    % 15 min discharge at 150 kW
% Second hour: standby with small load
P_new_cycle(61:120) = -10e3;    % 1 hour slow discharge at 10 kW

% Maximum power requirement
P_max_required = max(abs(P_new_cycle));
fprintf('  Maximum power required: %.0f kW\n', P_max_required/1000);

% Estimate energy requirement
% Must complete cycle starting at 50% SoC without going below 0%
E_net = sum(P_new_cycle) * mean(diff(t_new_cycle)); % Net energy [J]
fprintf('  Net cycle energy: %.2f kWh\n', E_net/3.6e6);

%% Define design space
% Design variable 1: Maximum speed
rpm_min = 20000;  % RPM
rpm_max = 50000;  % RPM
rpm_array = linspace(rpm_min, rpm_max, 8);
omega_array = rpm_array * (2*pi/60); % rad/s

% Design variable 2: Magnet thickness
t_mag_min = 0.002;  % 2 mm (minimum from spec)
t_mag_max = 0.008;  % 8 mm (reasonable maximum)
t_mag_array = linspace(t_mag_min, t_mag_max, 6); % m

fprintf('Design space:\n');
fprintf('  Speed range: %d - %d RPM (%d points)\n', rpm_min, rpm_max, length(rpm_array));
fprintf('  Magnet thickness: %.1f - %.1f mm (%d points)\n', ...
        t_mag_min*1000, t_mag_max*1000, length(t_mag_array));
fprintf('  Total designs to evaluate: %d\n\n', length(rpm_array)*length(t_mag_array));

%% Explore design space
n_designs = length(omega_array) * length(t_mag_array);
design_results = struct('omega_max', cell(n_designs,1), ...
                       'magnet_thickness', cell(n_designs,1), ...
                       'specific_power', cell(n_designs,1), ...
                       'specific_energy', cell(n_designs,1), ...
                       'efficiency', cell(n_designs,1), ...
                       'avg_loss', cell(n_designs,1), ...
                       'max_temp', cell(n_designs,1), ...
                       'feasible', cell(n_designs,1), ...
                       'dims', cell(n_designs,1), ...
                       'mass', cell(n_designs,1));

idx = 1;
fprintf('Evaluating designs...\n');

for i = 1:length(omega_array)
    for j = 1:length(t_mag_array)
        omega_max = omega_array(i);
        t_mag = t_mag_array(j);

        fprintf('  Design %d/%d: RPM=%.0f, t_mag=%.1f mm... ', ...
                idx, n_designs, omega_max*60/(2*pi), t_mag*1000);

        % Create design configuration
        design = createDesignConfig(omega_max, t_mag, E_net, P_max_required);

        % Simulate new storage cycle
        cycle_results = simulateStorageCycle(design, t_new_cycle, P_new_cycle, 0.5);

        % Check feasibility
        feasible = cycle_results.success && ...
                   max(cycle_results.T_rotor_sim) < 100 && ...
                   min(cycle_results.SoC_sim) >= 0;

        % Store results
        design_results(idx).omega_max = omega_max;
        design_results(idx).magnet_thickness = t_mag;
        design_results(idx).specific_power = design.P_rated / design.mass_total / 1000; % kW/kg
        [~, E_usable] = flywheelEnergy(omega_max, design.I_total);
        design_results(idx).specific_energy = E_usable / design.mass_total / 3.6e6; % kWh/kg
        design_results(idx).efficiency = cycle_results.efficiency;
        design_results(idx).avg_loss = mean(cycle_results.E_selfdischarge) / mean(diff(t_new_cycle));
        design_results(idx).max_temp = max(cycle_results.T_rotor_sim);
        design_results(idx).feasible = feasible;
        design_results(idx).dims = design.dims;
        design_results(idx).mass = design.mass_total;

        if feasible
            fprintf('FEASIBLE\n');
        else
            fprintf('NOT FEASIBLE (');
            if ~cycle_results.success
                fprintf('SoC constraint');
            elseif max(cycle_results.T_rotor_sim) >= 100
                fprintf('Temp=%.1fC', max(cycle_results.T_rotor_sim));
            end
            fprintf(')\n');
        end

        idx = idx + 1;
    end
end

%% Plot design trade-offs
fprintf('\nPlotting design trade-offs...\n');

% Extract data for feasible designs
feasible_idx = find([design_results.feasible]);
infeasible_idx = find(~[design_results.feasible]);

if isempty(feasible_idx)
    fprintf('  WARNING: No feasible designs found! Check constraints.\n');
    fprintf('  Showing all designs for reference.\n');
    feasible_idx = 1:n_designs;
end

sp_power_feas = [design_results(feasible_idx).specific_power];
sp_energy_feas = [design_results(feasible_idx).specific_energy];
eff_feas = [design_results(feasible_idx).efficiency] * 100;

% 3D scatter plot
figure('Name', 'Design Trade-offs');
if ~isempty(infeasible_idx)
    sp_power_inf = [design_results(infeasible_idx).specific_power];
    sp_energy_inf = [design_results(infeasible_idx).specific_energy];
    eff_inf = [design_results(infeasible_idx).efficiency] * 100;

    scatter3(sp_power_inf, sp_energy_inf, eff_inf, 100, 'x', 'MarkerEdgeColor', [0.7 0.7 0.7]);
    hold on;
end
scatter3(sp_power_feas, sp_energy_feas, eff_feas, 100, eff_feas, 'filled');
colorbar;
xlabel('Specific Power [kW/kg]');
ylabel('Specific Energy [kWh/kg]');
zlabel('Efficiency [%]');
title('Design Trade-offs: Specific Power vs Specific Energy vs Efficiency');
legend('Infeasible', 'Feasible', 'Location', 'best');
grid on;
view(45, 30);

saveas(gcf, '../results/design_tradeoffs_3d.fig');
saveas(gcf, '../results/design_tradeoffs_3d.png');

%% Select optimal design
% Selection criteria: maximize weighted combination of objectives
% Weights (adjust as needed)
w_sp_power = 0.3;
w_sp_energy = 0.3;
w_efficiency = 0.4;

% Normalize objectives to [0,1]
sp_power_norm = (sp_power_feas - min(sp_power_feas)) / (max(sp_power_feas) - min(sp_power_feas));
sp_energy_norm = (sp_energy_feas - min(sp_energy_feas)) / (max(sp_energy_feas) - min(sp_energy_feas));
eff_norm = (eff_feas - min(eff_feas)) / (max(eff_feas) - min(eff_feas));

% Calculate weighted score
scores = w_sp_power * sp_power_norm + w_sp_energy * sp_energy_norm + w_efficiency * eff_norm/100;

[~, best_idx_local] = max(scores);
best_idx = feasible_idx(best_idx_local);

optimal_design = design_results(best_idx);

fprintf('\n=== OPTIMAL DESIGN SELECTED ===\n');
fprintf('  Max speed: %.0f RPM\n', optimal_design.omega_max * 60/(2*pi));
fprintf('  Magnet thickness: %.1f mm\n', optimal_design.magnet_thickness * 1000);
fprintf('  Specific power: %.3f kW/kg\n', optimal_design.specific_power);
fprintf('  Specific energy: %.4f kWh/kg\n', optimal_design.specific_energy);
fprintf('  Efficiency: %.2f%%\n', optimal_design.efficiency * 100);
fprintf('  Max temperature: %.1f°C\n', optimal_design.max_temp);
fprintf('  Total mass: %.1f kg\n', optimal_design.mass);

%% Save results
save('../results/design_optimization_results.mat', 'design_results', ...
     'optimal_design', 't_new_cycle', 'P_new_cycle');

fprintf('\n=== DESIGN OPTIMIZATION COMPLETE ===\n');
fprintf('Results saved to ../results/\n\n');
