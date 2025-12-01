%% DELIVERABLE 2: NEW FLYWHEEL DESIGN STUDY
% Course: Mechanical Engineering Modeling
% Project: Flywheel Energy Storage System Analysis
%
% This script performs a design study to find an optimal flywheel design
% for the new 6-hour storage cycle. Design variables are:
%   - Magnet thickness (2-10 mm)
%   - Maximum rotational speed (limited by tip speed constraints)
%
% Objectives:
%   - Minimize losses
%   - Maximize specific power
%   - Maximize specific energy
%
% Constraints:
%   - Max composite tip speed: 900 m/s
%   - Max steel/PM tip speed: 175 m/s
%   - Max temperature: 100°C
%   - SoC > 0% throughout cycle
%
% Author: [Student Name]
% Date: 2025-11-25

clear; close all; clc;

%% Add paths
% Use official EE team functions (P-code) - add LAST to take precedence
addpath('../ee_functions');  % Custom implementations (lower priority)
addpath('../Project3_Functions');  % Official functions (higher priority)

%% ========================================================================
% SECTION 1: DESIGN STUDY PARAMETERS
% ========================================================================

fprintf('==============================================\n');
fprintf('DELIVERABLE 2: NEW FLYWHEEL DESIGN STUDY\n');
fprintf('==============================================\n\n');

% Fixed material properties (from Table 1)
rho_composite = 1600;    % Composite density [kg/m³]
rho_steel = 7850;        % Steel density [kg/m³]
rho_magnet = 7850;       % PM density [kg/m³]

% Safety limits
max_steel_tip_speed = 175;     % [m/s]
max_pm_tip_speed = 175;        % [m/s]
max_composite_tip_speed = 900; % [m/s]
max_temp = 100;                % [°C]
min_magnet_thickness = 0.002;  % 2 mm minimum

% Clearances (from Table 1)
axial_clearance = 0.020;       % 20 mm between components
radial_clearance_flywheel = 0.020;  % 20 mm flywheel to housing
radial_clearance_other = 0.001;     % 1 mm other components to housing

% Thermal parameters (from Table 1 in project spec)
epsilon_rotor = 0.4;     % Rotor emissivity
epsilon_housing = 0.9;   % Housing emissivity
sigma = 5.67e-8;         % Stefan-Boltzmann constant [W/(m²·K⁴)]
T_housing = 303;         % Housing temperature [K] = 30°C (from Appendix B)

% Current controller (from Appendix B)
Kp_current = 345;
Ki_current = 2149;

%% ========================================================================
% SECTION 2: DESIGN VARIABLE SWEEP
% ========================================================================

fprintf('Setting up design space exploration...\n\n');

% Design variables
magnet_thickness_range = linspace(0.002, 0.010, 9);   % 2-10 mm
max_speed_rpm_range = linspace(20000, 60000, 9);       % 20k-60k RPM

% Storage arrays for results
n_mag = length(magnet_thickness_range);
n_speed = length(max_speed_rpm_range);

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

%% ========================================================================
% SECTION 3: DESIGN SIZING FUNCTION
% ========================================================================

fprintf('Evaluating design space...\n');
fprintf('Progress: ');

for i = 1:n_mag
    for j = 1:n_speed
        % Design variables
        t_mag = magnet_thickness_range(i);
        omega_max_rpm = max_speed_rpm_range(j);
        omega_max = omega_max_rpm * 2*pi / 60;
        omega_min = omega_max / 2;  % 0% SoC at half speed

        results.magnet_thickness(i,j) = t_mag;
        results.max_speed_rpm(i,j) = omega_max_rpm;

        % ================================================================
        % STEP 1: Determine shaft diameter from PM tip speed limit
        % ================================================================
        % PM outer radius = shaft_radius + magnet_thickness
        % PM tip speed = (r_shaft + t_mag) * omega_max <= 175 m/s
        r_shaft_max = (max_pm_tip_speed / omega_max) - t_mag;

        % Also limited by steel tip speed
        r_shaft_steel = max_steel_tip_speed / omega_max;

        % Use the more restrictive
        r_shaft = min(r_shaft_max, r_shaft_steel);

        if r_shaft < 0.020  % Minimum shaft radius 20 mm
            results.viable(i,j) = 0;
            continue;
        end

        d_shaft = 2 * r_shaft;
        results.shaft_diameter(i,j) = d_shaft;

        % ================================================================
        % STEP 2: Determine flywheel diameter from composite tip speed
        % ================================================================
        r_flywheel = max_composite_tip_speed / omega_max;
        d_flywheel = 2 * r_flywheel;

        if d_flywheel < d_shaft + 0.050  % Need minimum flywheel thickness
            results.viable(i,j) = 0;
            continue;
        end

        results.flywheel_diameter(i,j) = d_flywheel;

        % ================================================================
        % STEP 3: Size motor for required power
        % ================================================================
        % Rated current = 1.0 pu (maximum available)
        I_rated_pu = 1.0;

        % Get magnetic shear stress
        shear = magneticShear(t_mag, I_rated_pu);

        % Motor length sizing: start with reasonable estimate, iterate
        % Torque = shear * pi * d_shaft * L_motor * (d_shaft/2)
        % Power = Torque * omega

        % Target minimum power: 450 kW for Team 16 cycle (peaks ~430 kW)
        P_target = 450000;  % [W]

        % Calculate required motor length
        torque_per_length = shear * pi * d_shaft * (d_shaft/2);
        L_motor_min = P_target / (torque_per_length * omega_max);

        % Round up to reasonable value, max 0.6 m for higher power requirements
        L_motor = max(0.150, min(0.600, ceil(L_motor_min * 100) / 100));
        results.motor_length(i,j) = L_motor;

        % Recalculate actual rated power
        motor_area = pi * d_shaft * L_motor;
        motor_radius = d_shaft / 2;
        torque_rated = shear * motor_area * motor_radius;
        P_rated = torque_rated * omega_max;
        results.rated_power(i,j) = P_rated;

        % ================================================================
        % STEP 4: Size flywheel for required energy storage
        % ================================================================
        % Target energy: enough for Team 16 cycle with margin
        % Cycle analysis shows: max charge excursion ~17 kWh, max discharge ~9.5 kWh
        % Starting at 50% SoC, need 0.5*capacity >= 17 kWh -> capacity >= 34 kWh
        % Target 40 kWh to provide adequate safety margin
        E_target = 40e3 * 3600;  % 40 kWh in Joules

        % Energy = 0.5 * I * (omega_max^2 - omega_min^2)
        % For hollow cylinder: I = 0.5 * m * (r_outer^2 + r_inner^2)
        % m = rho * pi * (r_outer^2 - r_inner^2) * L

        % Calculate required flywheel length for target energy
        r_outer = d_flywheel / 2;
        r_inner = d_shaft / 2;

        % Flywheel inertia per unit length
        V_per_length = pi * (r_outer^2 - r_inner^2);
        m_per_length = rho_composite * V_per_length;
        I_per_length = 0.5 * m_per_length * (r_outer^2 + r_inner^2);

        % Required inertia for target energy
        I_required = 2 * E_target / (omega_max^2 - omega_min^2);

        % Required flywheel length
        L_flywheel_min = I_required / I_per_length;

        % Round up, with limits (increased for higher energy requirements)
        L_flywheel = max(0.500, min(4.000, ceil(L_flywheel_min * 10) / 10));
        results.flywheel_length(i,j) = L_flywheel;

        % ================================================================
        % STEP 5: Calculate total mass and inertia
        % ================================================================
        % Flywheel mass
        V_flywheel = pi * (r_outer^2 - r_inner^2) * L_flywheel;
        m_flywheel = rho_composite * V_flywheel;

        % Shaft mass (total length includes motor, AMBs, clearances)
        L_shaft = L_flywheel + L_motor + 4*axial_clearance + 0.3;  % Extra for AMBs
        V_shaft = pi * r_inner^2 * L_shaft;
        m_shaft = rho_steel * V_shaft;

        % Magnet mass
        r_mag_outer = r_inner + t_mag;
        V_magnet = pi * (r_mag_outer^2 - r_inner^2) * L_motor;
        m_magnet = rho_magnet * V_magnet;

        % AMB rotor components (estimate 10% of shaft mass)
        m_amb = 0.10 * m_shaft;

        % Total mass
        m_total = m_flywheel + m_shaft + m_magnet + m_amb;
        results.total_mass(i,j) = m_total;

        % Total inertia
        I_flywheel = 0.5 * m_flywheel * (r_outer^2 + r_inner^2);
        I_shaft = 0.5 * m_shaft * r_inner^2;
        I_magnet = 0.5 * m_magnet * (r_mag_outer^2 + r_inner^2);
        I_total = I_flywheel + I_shaft + I_magnet;

        % ================================================================
        % STEP 6: Calculate specific power and energy
        % ================================================================
        E_max = 0.5 * I_total * omega_max^2;
        E_min = 0.5 * I_total * omega_min^2;
        E_stored = E_max - E_min;
        E_stored_Wh = E_stored / 3600;

        specific_power = (P_rated / 1000) / m_total;  % kW/kg
        specific_energy = E_stored_Wh / m_total;       % Wh/kg

        results.specific_power(i,j) = specific_power;
        results.specific_energy(i,j) = specific_energy;

        % ================================================================
        % STEP 7: Simulate storage cycle and calculate efficiency
        % ================================================================
        dt = 10.0;  % Time step [s]
        t_cycle = 0:dt:21600;  % 6-hour cycle

        omega_current = (omega_max + omega_min) / 2;  % Start at 50% SoC
        E_in = 0;
        E_out = 0;
        E_loss_total = 0;
        min_soc = 50;
        max_temp_cycle = 20;

        % Surface area for thermal calculation - correct two-surface enclosure model
        A_surface = 2*pi*r_outer*L_flywheel + 2*pi*r_outer^2;
        housing_clearance_ds = 0.020;  % 20mm radial clearance
        housing_inner_dia_ds = d_flywheel + 2*housing_clearance_ds;
        A_housing_ds = 2*pi*(housing_inner_dia_ds/2)*L_flywheel + 2*pi*(housing_inner_dia_ds/2)^2;
        rad_factor_ds = 1/epsilon_rotor + (A_surface/A_housing_ds)*(1/epsilon_housing - 1);

        for k = 1:length(t_cycle)
            % Grid power demand (using team-specific cycle)
            P_grid = team_16_cycle(t_cycle(k));

            % Current SoC and speed
            omega_rpm = omega_current * 60 / (2*pi);
            SoC = 100 * (omega_current - omega_min) / (omega_max - omega_min);

            if SoC < min_soc
                min_soc = SoC;
            end

            % Calculate current for this power
            I_pu = I_rated_pu * abs(P_grid) / P_rated;
            I_pu = min(I_pu, 1.0);

            % Calculate losses
            P_rotor = rotorLosses(t_mag, d_shaft, L_motor, I_pu, omega_rpm);
            P_stator = statorLosses(t_mag, d_shaft, L_motor, I_pu, omega_rpm);
            P_loss = P_rotor + P_stator;
            E_loss_total = E_loss_total + P_loss * dt;

            % Temperature using two-surface enclosure radiation model
            % Q = sigma * A * (T_rotor^4 - T_housing^4) / rad_factor
            T_rotor = (T_housing^4 + P_rotor*rad_factor_ds/(sigma*A_surface))^(1/4);
            T_celsius = T_rotor - 273;
            if T_celsius > max_temp_cycle
                max_temp_cycle = T_celsius;
            end

            % Update energy
            E_current = 0.5 * I_total * omega_current^2;
            P_net = P_grid + P_loss;
            dE = -P_net * dt;
            E_current = E_current + dE;

            % Update speed
            if E_current > 0
                omega_current = sqrt(2 * E_current / I_total);
            end
            omega_current = max(omega_min*0.9, min(omega_max*1.05, omega_current));

            % Track energy flows
            if P_grid > 0
                E_out = E_out + P_grid * dt;
            else
                E_in = E_in + abs(P_grid) * dt;
            end
        end

        results.min_soc(i,j) = min_soc;
        results.max_temp(i,j) = max_temp_cycle;

        % Calculate efficiency
        % Final SoC
        SoC_final = 100 * (omega_current - omega_min) / (omega_max - omega_min);
        E_recovery = max(0, 0.5 * I_total * ((omega_max+omega_min)/2)^2 - 0.5 * I_total * omega_current^2);

        if (E_in + E_recovery) > 0
            efficiency = (E_out / (E_in + E_recovery)) * 100;
        else
            efficiency = 0;
        end
        results.efficiency(i,j) = efficiency;

        % ================================================================
        % STEP 8: Check viability
        % ================================================================
        viable = 1;

        % Temperature constraint
        if max_temp_cycle > max_temp
            viable = 0;
        end

        % SoC constraint (must stay above 0%)
        if min_soc < 0
            viable = 0;
        end

        % Reasonable efficiency (relax lower bound for challenging cycles)
        if efficiency < 70 || efficiency > 105
            viable = 0;
        end

        % Also check that rated power can meet cycle demand (Team 16 peaks ~430 kW)
        if P_rated < 430000  % Need at least 430 kW for the Team 16 cycle
            viable = 0;
        end

        results.viable(i,j) = viable;
    end
    fprintf('.');
end
fprintf(' Done!\n\n');

%% ========================================================================
% SECTION 4: RESULTS VISUALIZATION
% ========================================================================

fprintf('==============================================\n');
fprintf('DESIGN SPACE ANALYSIS RESULTS\n');
fprintf('==============================================\n\n');

% Count viable designs
n_viable = sum(results.viable(:));
fprintf('Viable designs: %d out of %d\n\n', n_viable, n_mag*n_speed);

% Create meshgrid for plotting
[MAG, SPD] = meshgrid(magnet_thickness_range*1000, max_speed_rpm_range/1000);

% Figure 1: Specific Power vs Specific Energy (colored by efficiency)
figure('Name', 'Design Trade-offs', 'Position', [100 100 1200 500]);

subplot(1,2,1);
viable_mask = results.viable' == 1;
scatter(results.specific_energy(viable_mask'), results.specific_power(viable_mask'), ...
    80, results.efficiency(viable_mask'), 'filled');
colorbar;
colormap(gca, 'parula');
caxis([90 100]);
xlabel('Specific Energy [Wh/kg]', 'FontSize', 12);
ylabel('Specific Power [kW/kg]', 'FontSize', 12);
title('Viable Designs: Specific Power vs Energy', 'FontSize', 14);
grid on;
hold on;

% Mark baseline
baseline_sp = 0.833;
baseline_se = 33.217;
plot(baseline_se, baseline_sp, 'rp', 'MarkerSize', 15, 'MarkerFaceColor', 'r');
text(baseline_se+1, baseline_sp, 'Baseline', 'FontSize', 10);

subplot(1,2,2);
scatter3(results.specific_energy(viable_mask'), results.specific_power(viable_mask'), ...
    results.efficiency(viable_mask'), 80, results.max_temp(viable_mask'), 'filled');
colorbar;
xlabel('Specific Energy [Wh/kg]', 'FontSize', 11);
ylabel('Specific Power [kW/kg]', 'FontSize', 11);
zlabel('Efficiency [%]', 'FontSize', 11);
title('3D Trade-off Space (color = max temp)', 'FontSize', 13);
grid on;
view(45, 30);

saveas(gcf, 'part2a_design_tradeoffs.fig');
saveas(gcf, 'part2a_design_tradeoffs.png');

% Figure 2: Design parameter effects
figure('Name', 'Design Parameters', 'Position', [100 100 1200 800]);

subplot(2,2,1);
contourf(MAG, SPD, results.specific_power', 20);
colorbar;
xlabel('Magnet Thickness [mm]', 'FontSize', 11);
ylabel('Max Speed [krpm]', 'FontSize', 11);
title('Specific Power [kW/kg]', 'FontSize', 12);

subplot(2,2,2);
contourf(MAG, SPD, results.specific_energy', 20);
colorbar;
xlabel('Magnet Thickness [mm]', 'FontSize', 11);
ylabel('Max Speed [krpm]', 'FontSize', 11);
title('Specific Energy [Wh/kg]', 'FontSize', 12);

subplot(2,2,3);
eff_plot = results.efficiency';
eff_plot(results.viable' == 0) = NaN;
contourf(MAG, SPD, eff_plot, 20);
colorbar;
xlabel('Magnet Thickness [mm]', 'FontSize', 11);
ylabel('Max Speed [krpm]', 'FontSize', 11);
title('Cycle Efficiency [%] (viable only)', 'FontSize', 12);

subplot(2,2,4);
contourf(MAG, SPD, results.max_temp', 20);
colorbar;
hold on;
contour(MAG, SPD, results.max_temp', [100 100], 'r-', 'LineWidth', 2);
xlabel('Magnet Thickness [mm]', 'FontSize', 11);
ylabel('Max Speed [krpm]', 'FontSize', 11);
title('Max Temperature [°C] (red = 100°C limit)', 'FontSize', 12);

saveas(gcf, 'part2a_design_parameters.fig');
saveas(gcf, 'part2a_design_parameters.png');

%% ========================================================================
% SECTION 5: OPTIMAL DESIGN SELECTION
% ========================================================================

fprintf('==============================================\n');
fprintf('OPTIMAL DESIGN SELECTION\n');
fprintf('==============================================\n\n');

% Find Pareto-optimal designs
% Objective: maximize specific power, specific energy, efficiency
% while minimizing temperature

viable_idx = find(results.viable(:) == 1);
if isempty(viable_idx)
    error('No viable designs found!');
end

% Calculate composite score (weighted objectives)
% Normalize each objective to 0-1 range
sp_norm = (results.specific_power - min(results.specific_power(:))) / ...
    (max(results.specific_power(:)) - min(results.specific_power(:)));
se_norm = (results.specific_energy - min(results.specific_energy(:))) / ...
    (max(results.specific_energy(:)) - min(results.specific_energy(:)));
eff_norm = (results.efficiency - 90) / 10;  % Assume 90-100% range
temp_norm = 1 - (results.max_temp - 20) / 80;  % Lower is better

% Composite score (weighted sum)
score = 0.3 * sp_norm + 0.3 * se_norm + 0.25 * eff_norm + 0.15 * temp_norm;
score(results.viable == 0) = -Inf;

% Find best design
[best_score, best_idx] = max(score(:));
[best_i, best_j] = ind2sub(size(score), best_idx);

% Extract optimal design parameters
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

fprintf('OPTIMAL DESIGN SPECIFICATIONS:\n\n');
fprintf('Design Variables:\n');
fprintf('  Magnet thickness: %.1f mm\n', optimal.magnet_thickness*1000);
fprintf('  Max rotational speed: %.0f RPM\n', optimal.max_speed_rpm);
fprintf('\nDerived Dimensions:\n');
fprintf('  Flywheel diameter: %.1f mm\n', optimal.flywheel_diameter*1000);
fprintf('  Flywheel length: %.1f mm\n', optimal.flywheel_length*1000);
fprintf('  Shaft diameter: %.1f mm\n', optimal.shaft_diameter*1000);
fprintf('  Motor length: %.1f mm\n', optimal.motor_length*1000);
fprintf('\nPerformance Metrics:\n');
fprintf('  Total mass: %.1f kg\n', optimal.total_mass);
fprintf('  Rated power: %.1f kW\n', optimal.rated_power/1000);
fprintf('  Specific power: %.3f kW/kg\n', optimal.specific_power);
fprintf('  Specific energy: %.2f Wh/kg\n', optimal.specific_energy);
fprintf('  Cycle efficiency: %.2f%%\n', optimal.efficiency);
fprintf('  Max temperature: %.1f °C\n', optimal.max_temp);
fprintf('  Min SoC during cycle: %.1f%%\n', optimal.min_soc);

%% ========================================================================
% SECTION 6: COMPARISON TO BASELINE
% ========================================================================

fprintf('\n==============================================\n');
fprintf('COMPARISON TO BASELINE SYSTEM\n');
fprintf('==============================================\n\n');

fprintf('                      Baseline    New Design    Change\n');
fprintf('-------------------------------------------------------\n');
fprintf('Specific Power [kW/kg]  %.3f       %.3f        %+.1f%%\n', ...
    baseline_sp, optimal.specific_power, ...
    100*(optimal.specific_power - baseline_sp)/baseline_sp);
fprintf('Specific Energy [Wh/kg] %.2f       %.2f        %+.1f%%\n', ...
    baseline_se, optimal.specific_energy, ...
    100*(optimal.specific_energy - baseline_se)/baseline_se);
fprintf('Cycle Efficiency [%%]    97.4        %.2f        %+.1f%%\n', ...
    optimal.efficiency, optimal.efficiency - 97.4);
fprintf('Max Temperature [°C]    63.8        %.1f        %+.1f\n', ...
    optimal.max_temp, optimal.max_temp - 63.8);

% Save optimal design to file for Deliverable 3
save('optimal_design.mat', 'optimal', 'results');

fprintf('\nOptimal design saved to optimal_design.mat\n');
fprintf('==============================================\n');

%% ========================================================================
% SECTION 7: DETAILED CYCLE SIMULATION OF OPTIMAL DESIGN
% ========================================================================

fprintf('\n==============================================\n');
fprintf('OPTIMAL DESIGN CYCLE SIMULATION\n');
fprintf('==============================================\n\n');

% Recalculate with fine time resolution for plotting
dt = 5.0;
t_cycle = 0:dt:21600;

% Recalculate properties
t_mag = optimal.magnet_thickness;
omega_max = optimal.max_speed_rpm * 2*pi / 60;
omega_min = omega_max / 2;
d_shaft = optimal.shaft_diameter;
L_motor = optimal.motor_length;
d_flywheel = optimal.flywheel_diameter;
L_flywheel = optimal.flywheel_length;

% Inertia
r_outer = d_flywheel / 2;
r_inner = d_shaft / 2;
r_mag = r_inner + t_mag;
V_flywheel = pi * (r_outer^2 - r_inner^2) * L_flywheel;
m_flywheel = rho_composite * V_flywheel;
L_shaft = L_flywheel + L_motor + 4*axial_clearance + 0.3;
V_shaft = pi * r_inner^2 * L_shaft;
m_shaft = rho_steel * V_shaft;
V_magnet = pi * (r_mag^2 - r_inner^2) * L_motor;
m_magnet = rho_magnet * V_magnet;

I_flywheel = 0.5 * m_flywheel * (r_outer^2 + r_inner^2);
I_shaft = 0.5 * m_shaft * r_inner^2;
I_magnet = 0.5 * m_magnet * (r_mag^2 + r_inner^2);
I_total = I_flywheel + I_shaft + I_magnet;

A_surface = 2*pi*r_outer*L_flywheel + 2*pi*r_outer^2;
% Correct two-surface enclosure radiation model
housing_clearance_opt = 0.020;  % 20mm radial clearance
housing_inner_dia_opt = d_flywheel + 2*housing_clearance_opt;
A_housing_opt = 2*pi*(housing_inner_dia_opt/2)*L_flywheel + 2*pi*(housing_inner_dia_opt/2)^2;
rad_factor_opt = 1/epsilon_rotor + (A_surface/A_housing_opt)*(1/epsilon_housing - 1);

I_rated_pu = 1.0;  % Maximum available current
shear = magneticShear(t_mag, I_rated_pu);
P_rated = shear * pi * d_shaft * L_motor * (d_shaft/2) * omega_max;

% Simulate
omega_current = (omega_max + omega_min) / 2;
omega_array = zeros(size(t_cycle));
SoC_array = zeros(size(t_cycle));
power_grid_array = zeros(size(t_cycle));
power_loss_array = zeros(size(t_cycle));
temp_array = zeros(size(t_cycle));

for k = 1:length(t_cycle)
    P_grid = team_16_cycle(t_cycle(k));
    power_grid_array(k) = P_grid;

    omega_rpm = omega_current * 60 / (2*pi);
    SoC_array(k) = 100 * (omega_current - omega_min) / (omega_max - omega_min);
    omega_array(k) = omega_current;

    I_pu = I_rated_pu * abs(P_grid) / P_rated;
    I_pu = min(I_pu, 1.0);

    P_rotor = rotorLosses(t_mag, d_shaft, L_motor, I_pu, omega_rpm);
    P_stator = statorLosses(t_mag, d_shaft, L_motor, I_pu, omega_rpm);
    power_loss_array(k) = P_rotor + P_stator;

    % Two-surface enclosure radiation model
    T_rotor = (T_housing^4 + P_rotor*rad_factor_opt/(sigma*A_surface))^(1/4);
    temp_array(k) = T_rotor - 273;

    E_current = 0.5 * I_total * omega_current^2;
    P_net = P_grid + P_rotor + P_stator;
    dE = -P_net * dt;
    E_current = E_current + dE;

    if E_current > 0
        omega_current = sqrt(2 * E_current / I_total);
    end
    omega_current = max(omega_min*0.9, min(omega_max*1.05, omega_current));
end

% Plot cycle simulation
figure('Name', 'Optimal Design Cycle', 'Position', [100 100 1000 800]);

subplot(2,2,1);
plot(t_cycle/3600, power_grid_array/1000, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time [hours]', 'FontSize', 11);
ylabel('Grid Power [kW]', 'FontSize', 11);
title('Team 16 Storage Cycle Power Demand', 'FontSize', 12);

subplot(2,2,2);
plot(t_cycle/3600, SoC_array, 'r-', 'LineWidth', 2);
hold on;
yline(0, 'k--', 'LineWidth', 1.5);
grid on;
xlabel('Time [hours]', 'FontSize', 11);
ylabel('State of Charge [%]', 'FontSize', 11);
title('SoC During Cycle', 'FontSize', 12);
ylim([-5 105]);

subplot(2,2,3);
plot(t_cycle/3600, power_loss_array/1000, 'k-', 'LineWidth', 1.5);
grid on;
xlabel('Time [hours]', 'FontSize', 11);
ylabel('Total Losses [kW]', 'FontSize', 11);
title('Motor Losses', 'FontSize', 12);

subplot(2,2,4);
plot(t_cycle/3600, temp_array, 'm-', 'LineWidth', 2);
hold on;
yline(100, 'r--', 'LineWidth', 1.5, 'Label', 'Max Safe');
grid on;
xlabel('Time [hours]', 'FontSize', 11);
ylabel('Temperature [°C]', 'FontSize', 11);
title('Rotor Temperature', 'FontSize', 12);

saveas(gcf, 'part2b_optimal_cycle.fig');
saveas(gcf, 'part2b_optimal_cycle.png');

fprintf('Cycle simulation plots saved.\n');
fprintf('==============================================\n');
