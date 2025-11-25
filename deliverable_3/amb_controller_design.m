%% DELIVERABLE 3: AMB CONTROLLER DESIGN
% Course: Mechanical Engineering Modeling
% Project: Flywheel Energy Storage System Analysis
%
% This script designs AMB controllers for the optimal flywheel design
% from Deliverable 2 and compares performance to the baseline system.
%
% Tasks:
%   3a. Provide transfer functions for position and force controllers
%   3b. Compare AMB step response to baseline
%   3c. Compare dynamic stiffness and rotor runout to baseline
%
% Author: [Student Name]
% Date: 2025-11-25

clear; close all; clc;

%% Add paths
addpath('../ee_functions');
addpath('../Project3_Functions');

%% ========================================================================
% SECTION 1: LOAD OPTIMAL DESIGN FROM DELIVERABLE 2
% ========================================================================

fprintf('==============================================\n');
fprintf('DELIVERABLE 3: AMB CONTROLLER DESIGN\n');
fprintf('==============================================\n\n');

% Load optimal design from Deliverable 2
load('../deliverable_2/optimal_design.mat');

fprintf('Loading optimal design from Deliverable 2...\n');
fprintf('  Magnet thickness: %.1f mm\n', optimal.magnet_thickness*1000);
fprintf('  Max speed: %.0f RPM\n', optimal.max_speed_rpm);
fprintf('  Total mass: %.1f kg\n', optimal.total_mass);
fprintf('  Flywheel diameter: %.1f mm\n', optimal.flywheel_diameter*1000);
fprintf('  Shaft diameter: %.1f mm\n', optimal.shaft_diameter*1000);
fprintf('\n');

%% ========================================================================
% SECTION 2: BASELINE SYSTEM PARAMETERS (FROM DELIVERABLE 1)
% ========================================================================

fprintf('Loading baseline system parameters...\n');

% Baseline parameters (from Appendix B)
baseline = struct();
baseline.flywheel_length = 1.000;
baseline.flywheel_diameter = 0.430;
baseline.motor_length = 0.250;
baseline.shaft_diameter = 0.084;
baseline.magnet_thickness = 0.006;
baseline.max_speed_rpm = 40000;
baseline.amb_rated_force = 5780;

% Material densities
rho_composite = 1600;
rho_steel = 7850;
rho_magnet = 7850;

% Calculate baseline mass and inertia
r_outer_bl = baseline.flywheel_diameter / 2;
r_inner_bl = baseline.shaft_diameter / 2;
V_flywheel_bl = pi * (r_outer_bl^2 - r_inner_bl^2) * baseline.flywheel_length;
m_flywheel_bl = rho_composite * V_flywheel_bl;

L_shaft_bl = baseline.flywheel_length + baseline.motor_length + 0.5;
V_shaft_bl = pi * r_inner_bl^2 * L_shaft_bl;
m_shaft_bl = rho_steel * V_shaft_bl;

r_mag_bl = r_inner_bl + baseline.magnet_thickness;
V_mag_bl = pi * (r_mag_bl^2 - r_inner_bl^2) * baseline.motor_length;
m_mag_bl = rho_magnet * V_mag_bl;

baseline.total_mass = m_flywheel_bl + m_shaft_bl + m_mag_bl + 0.1*m_shaft_bl;

I_flywheel_bl = 0.5 * m_flywheel_bl * (r_outer_bl^2 + r_inner_bl^2);
I_shaft_bl = 0.5 * m_shaft_bl * r_inner_bl^2;
I_mag_bl = 0.5 * m_mag_bl * (r_mag_bl^2 + r_inner_bl^2);
baseline.I_total = I_flywheel_bl + I_shaft_bl + I_mag_bl;

fprintf('  Baseline mass: %.1f kg\n', baseline.total_mass);
fprintf('  Baseline AMB rated force: %.0f N\n', baseline.amb_rated_force);

%% ========================================================================
% SECTION 3: NEW SYSTEM AMB SIZING
% ========================================================================

fprintf('\nSizing AMB for new design...\n');

% AMB rated force = 2 × rotating group weight (per project requirements)
g = 9.81;  % Gravity [m/s²]
new_amb_force = 2 * optimal.total_mass * g;

fprintf('  New system mass: %.1f kg\n', optimal.total_mass);
fprintf('  New AMB rated force: %.0f N\n', new_amb_force);

% Get AMB parameters for both systems
amb_baseline = ambParameters(baseline.shaft_diameter, baseline.amb_rated_force);
amb_new = ambParameters(optimal.shaft_diameter, new_amb_force);

fprintf('\nAMB Parameter Comparison:\n');
fprintf('                          Baseline      New Design\n');
fprintf('  Stiffness constant:     %.2e    %.2e  [N/m]\n', ...
    amb_baseline.stiffnessConstant, amb_new.stiffnessConstant);
fprintf('  Force constant:         %.2f        %.2f        [N/A]\n', ...
    amb_baseline.forceConstant, amb_new.forceConstant);
fprintf('  Coil inductance:        %.4f        %.4f      [H]\n', ...
    amb_baseline.coilInductance, amb_new.coilInductance);
fprintf('  Coil resistance:        %.3f         %.3f       [Ohms]\n', ...
    amb_baseline.coilResistance, amb_new.coilResistance);
fprintf('  Axial length:           %.3f         %.3f       [m]\n', ...
    amb_baseline.axialLength, amb_new.axialLength);

%% ========================================================================
% SECTION 4: CONTROLLER DESIGN
% ========================================================================

fprintf('\n==============================================\n');
fprintf('PART 3a: Controller Transfer Functions\n');
fprintf('==============================================\n\n');

% Current controller (given in project specification)
% G_ci(s) = 258.75 + 1611.54/s = (258.75*s + 1611.54)/s
Kp_current = 258.75;
Ki_current = 1611.54;

fprintf('Current Controller (given):\n');
fprintf('  G_ci(s) = Kp + Ki/s = %.2f + %.2f/s\n', Kp_current, Ki_current);
fprintf('  Transfer function: G_ci(s) = (%.2f*s + %.2f) / s\n\n', Kp_current, Ki_current);

% Design position controllers for both systems
% Target: critically damped or slightly underdamped response
% Similar approach to Deliverable 1 but optimized for each system

% Extract AMB physical parameters
K_s_baseline = -amb_baseline.stiffnessConstant;  % Negative stiffness (destabilizing)
K_i_baseline = amb_baseline.forceConstant;

K_s_new = -amb_new.stiffnessConstant;
K_i_new = amb_new.forceConstant;

% Design targets
omega_n_target = 2*pi*50;  % 50 Hz natural frequency
zeta_target = 0.7;         % Damping ratio (slight underdamping)

% BASELINE Position Controller Design
% PD controller: u = Kp*e + Kd*de/dt
% Closed-loop poles at s = -zeta*omega_n ± j*omega_n*sqrt(1-zeta^2)
Kp_pos_baseline = omega_n_target^2 * baseline.total_mass / K_i_baseline;
Kd_pos_baseline = 2*zeta_target*omega_n_target * baseline.total_mass / K_i_baseline;

% NEW SYSTEM Position Controller Design
Kp_pos_new = omega_n_target^2 * optimal.total_mass / K_i_new;
Kd_pos_new = 2*zeta_target*omega_n_target * optimal.total_mass / K_i_new;

fprintf('Position Controller Design (PD type):\n');
fprintf('  G_pos(s) = Kp + Kd*s\n\n');
fprintf('                          Baseline      New Design\n');
fprintf('  Proportional (Kp):      %.2f      %.2f      [A/m]\n', ...
    Kp_pos_baseline, Kp_pos_new);
fprintf('  Derivative (Kd):        %.4f      %.4f    [A·s/m]\n', ...
    Kd_pos_baseline, Kd_pos_new);

% Calculate resulting closed-loop characteristics
omega_n_cl_baseline = sqrt((Kp_pos_baseline * K_i_baseline + K_s_baseline) / baseline.total_mass);
omega_n_cl_new = sqrt((Kp_pos_new * K_i_new + K_s_new) / optimal.total_mass);

zeta_cl_baseline = (Kd_pos_baseline * K_i_baseline) / (2 * baseline.total_mass * omega_n_cl_baseline);
zeta_cl_new = (Kd_pos_new * K_i_new) / (2 * optimal.total_mass * omega_n_cl_new);

fprintf('\nClosed-Loop Characteristics:\n');
fprintf('                          Baseline      New Design\n');
fprintf('  Natural frequency:      %.1f Hz       %.1f Hz\n', ...
    omega_n_cl_baseline/(2*pi), omega_n_cl_new/(2*pi));
fprintf('  Damping ratio:          %.2f          %.2f\n', ...
    zeta_cl_baseline, zeta_cl_new);

%% ========================================================================
% SECTION 5: TRANSFER FUNCTION DERIVATION
% ========================================================================

fprintf('\n==============================================\n');
fprintf('PART 3a: Complete Transfer Functions\n');
fprintf('==============================================\n\n');

fprintf('AMB SYSTEM TRANSFER FUNCTIONS\n\n');

fprintf('1. Current Controller (PI):\n');
fprintf('   G_ci(s) = %.2f + %.2f/s\n\n', Kp_current, Ki_current);

fprintf('2. Position Controller (PD) - Baseline:\n');
fprintf('   G_pos(s) = %.2f + %.4f*s\n', Kp_pos_baseline, Kd_pos_baseline);
fprintf('   Transfer function: G_pos(s) = (%.4f*s + %.2f) / 1\n\n', ...
    Kd_pos_baseline, Kp_pos_baseline);

fprintf('3. Position Controller (PD) - New Design:\n');
fprintf('   G_pos(s) = %.2f + %.4f*s\n', Kp_pos_new, Kd_pos_new);
fprintf('   Transfer function: G_pos(s) = (%.4f*s + %.2f) / 1\n\n', ...
    Kd_pos_new, Kp_pos_new);

fprintf('4. Plant Model (AMB + Rotor):\n');
fprintf('   G_plant(s) = Ki / (m*s^2 - Ks)\n');
fprintf('   Baseline: G_plant(s) = %.2f / (%.2f*s^2 - %.2e)\n', ...
    K_i_baseline, baseline.total_mass, -K_s_baseline);
fprintf('   New:      G_plant(s) = %.2f / (%.2f*s^2 - %.2e)\n\n', ...
    K_i_new, optimal.total_mass, -K_s_new);

%% ========================================================================
% SECTION 6: STEP RESPONSE SIMULATION
% ========================================================================

fprintf('==============================================\n');
fprintf('PART 3b: AMB Step Response Comparison\n');
fprintf('==============================================\n\n');

% Simulation parameters
dt = 1e-5;  % Time step [s]
t_sim = 0:dt:0.05;  % 50 ms simulation

% Step disturbance = 10% of rated force
F_step_baseline = 0.10 * baseline.amb_rated_force;
F_step_new = 0.10 * new_amb_force;

fprintf('Simulating step response...\n');
fprintf('  Baseline disturbance: %.1f N (10%% of %.0f N)\n', F_step_baseline, baseline.amb_rated_force);
fprintf('  New design disturbance: %.1f N (10%% of %.0f N)\n', F_step_new, new_amb_force);

% Storage arrays
x_baseline = zeros(2, length(t_sim));  % [at 0 RPM; at max RPM]
x_new = zeros(2, length(t_sim));
i_baseline = zeros(2, length(t_sim));
i_new = zeros(2, length(t_sim));
f_baseline = zeros(2, length(t_sim));
f_new = zeros(2, length(t_sim));

% Simulate both systems at two conditions: 0 RPM and max RPM
omega_conditions = [0, baseline.max_speed_rpm * 2*pi/60; ...
                   0, optimal.max_speed_rpm * 2*pi/60];

for cond = 1:2
    % BASELINE simulation
    x = 0; v = 0; i = 0;
    tau_i = 1 / Kp_current;

    for k = 1:length(t_sim)
        i_cmd = -Kp_pos_baseline * x - Kd_pos_baseline * v;
        di_dt = (i_cmd - i) / tau_i;
        F_amb = K_i_baseline * i - K_s_baseline * x;
        a = (F_amb + F_step_baseline) / baseline.total_mass;

        v = v + a * dt;
        x = x + v * dt;
        i = i + di_dt * dt;

        x_baseline(cond, k) = x;
        i_baseline(cond, k) = i;
        f_baseline(cond, k) = F_amb;
    end

    % NEW SYSTEM simulation
    x = 0; v = 0; i = 0;

    for k = 1:length(t_sim)
        i_cmd = -Kp_pos_new * x - Kd_pos_new * v;
        di_dt = (i_cmd - i) / tau_i;
        F_amb = K_i_new * i - K_s_new * x;
        a = (F_amb + F_step_new) / optimal.total_mass;

        v = v + a * dt;
        x = x + v * dt;
        i = i + di_dt * dt;

        x_new(cond, k) = x;
        i_new(cond, k) = i;
        f_new(cond, k) = F_amb;
    end
end

% Create comparison plots
figure('Name', 'AMB Step Response Comparison', 'Position', [100 100 1200 800]);

% Position response at 0 RPM
subplot(2,3,1);
plot(t_sim*1000, x_baseline(1,:)*1e6, 'b-', 'LineWidth', 2); hold on;
plot(t_sim*1000, x_new(1,:)*1e6, 'r--', 'LineWidth', 2);
grid on;
xlabel('Time [ms]', 'FontSize', 11);
ylabel('Position [\mum]', 'FontSize', 11);
title('Position at 0 RPM', 'FontSize', 12);
legend('Baseline', 'New Design', 'Location', 'best');

% Position response at max RPM
subplot(2,3,4);
plot(t_sim*1000, x_baseline(2,:)*1e6, 'b-', 'LineWidth', 2); hold on;
plot(t_sim*1000, x_new(2,:)*1e6, 'r--', 'LineWidth', 2);
grid on;
xlabel('Time [ms]', 'FontSize', 11);
ylabel('Position [\mum]', 'FontSize', 11);
title('Position at Max RPM', 'FontSize', 12);
legend('Baseline', 'New Design', 'Location', 'best');

% Current response at 0 RPM
subplot(2,3,2);
plot(t_sim*1000, i_baseline(1,:), 'b-', 'LineWidth', 2); hold on;
plot(t_sim*1000, i_new(1,:), 'r--', 'LineWidth', 2);
grid on;
xlabel('Time [ms]', 'FontSize', 11);
ylabel('Current [A]', 'FontSize', 11);
title('Current at 0 RPM', 'FontSize', 12);
legend('Baseline', 'New Design', 'Location', 'best');

% Current response at max RPM
subplot(2,3,5);
plot(t_sim*1000, i_baseline(2,:), 'b-', 'LineWidth', 2); hold on;
plot(t_sim*1000, i_new(2,:), 'r--', 'LineWidth', 2);
grid on;
xlabel('Time [ms]', 'FontSize', 11);
ylabel('Current [A]', 'FontSize', 11);
title('Current at Max RPM', 'FontSize', 12);
legend('Baseline', 'New Design', 'Location', 'best');

% Force response at 0 RPM
subplot(2,3,3);
plot(t_sim*1000, f_baseline(1,:), 'b-', 'LineWidth', 2); hold on;
plot(t_sim*1000, f_new(1,:), 'r--', 'LineWidth', 2);
grid on;
xlabel('Time [ms]', 'FontSize', 11);
ylabel('AMB Force [N]', 'FontSize', 11);
title('Force at 0 RPM', 'FontSize', 12);
legend('Baseline', 'New Design', 'Location', 'best');

% Force response at max RPM
subplot(2,3,6);
plot(t_sim*1000, f_baseline(2,:), 'b-', 'LineWidth', 2); hold on;
plot(t_sim*1000, f_new(2,:), 'r--', 'LineWidth', 2);
grid on;
xlabel('Time [ms]', 'FontSize', 11);
ylabel('AMB Force [N]', 'FontSize', 11);
title('Force at Max RPM', 'FontSize', 12);
legend('Baseline', 'New Design', 'Location', 'best');

saveas(gcf, 'part3b_step_response.fig');
saveas(gcf, 'part3b_step_response.png');

% Calculate performance metrics
settling_idx_baseline = find(abs(x_baseline(1,:) - x_baseline(1,end)) < 0.02*max(abs(x_baseline(1,:))), 1);
settling_idx_new = find(abs(x_new(1,:) - x_new(1,end)) < 0.02*max(abs(x_new(1,:))), 1);

settling_time_baseline = t_sim(settling_idx_baseline) * 1000;
settling_time_new = t_sim(settling_idx_new) * 1000;

peak_disp_baseline = max(abs(x_baseline(1,:))) * 1e6;
peak_disp_new = max(abs(x_new(1,:))) * 1e6;

fprintf('\nStep Response Performance:\n');
fprintf('                          Baseline      New Design\n');
fprintf('  Peak displacement:      %.2f µm      %.2f µm\n', peak_disp_baseline, peak_disp_new);
fprintf('  Settling time (2%%):     %.1f ms       %.1f ms\n', settling_time_baseline, settling_time_new);

%% ========================================================================
% SECTION 7: DYNAMIC STIFFNESS
% ========================================================================

fprintf('\n==============================================\n');
fprintf('PART 3c: Dynamic Stiffness Comparison\n');
fprintf('==============================================\n\n');

% Frequency range
freq = logspace(0, 3, 200);  % 1-1000 Hz
omega = 2*pi*freq;

% Calculate dynamic stiffness for both systems
stiff_baseline_radial = zeros(size(omega));
stiff_new_radial = zeros(size(omega));
stiff_baseline_tilt = zeros(size(omega));
stiff_new_tilt = zeros(size(omega));

% Effective parameters
k_eff_baseline = Kp_pos_baseline * K_i_baseline - K_s_baseline;
c_eff_baseline = Kd_pos_baseline * K_i_baseline;

k_eff_new = Kp_pos_new * K_i_new - K_s_new;
c_eff_new = Kd_pos_new * K_i_new;

% AMB separation distances (estimate)
L_amb_baseline = baseline.flywheel_length + 0.3;
L_amb_new = optimal.flywheel_length + 0.3;

% Tilting effective mass
m_eff_tilt_baseline = baseline.I_total / L_amb_baseline^2;
% Calculate new system inertia (not stored in optimal struct)
r_outer_new_stiff = optimal.flywheel_diameter / 2;
r_inner_new_stiff = optimal.shaft_diameter / 2;
V_fw_new_stiff = pi * (r_outer_new_stiff^2 - r_inner_new_stiff^2) * optimal.flywheel_length;
m_fw_new_stiff = rho_composite * V_fw_new_stiff;
I_flywheel_new_stiff = 0.5 * m_fw_new_stiff * (r_outer_new_stiff^2 + r_inner_new_stiff^2);
I_total_new_stiff = I_flywheel_new_stiff;  % Flywheel dominates
m_eff_tilt_new = I_total_new_stiff / L_amb_new^2;

% Calculate inertia for new design
r_outer_new = optimal.flywheel_diameter / 2;
r_inner_new = optimal.shaft_diameter / 2;
V_flywheel_new = pi * (r_outer_new^2 - r_inner_new^2) * optimal.flywheel_length;
m_flywheel_new = rho_composite * V_flywheel_new;
L_shaft_new = optimal.flywheel_length + optimal.motor_length + 0.5;
V_shaft_new = pi * r_inner_new^2 * L_shaft_new;
m_shaft_new = rho_steel * V_shaft_new;
r_mag_new = r_inner_new + optimal.magnet_thickness;
V_mag_new = pi * (r_mag_new^2 - r_inner_new^2) * optimal.motor_length;
m_mag_new = rho_magnet * V_mag_new;

I_flywheel_new = 0.5 * m_flywheel_new * (r_outer_new^2 + r_inner_new^2);
I_shaft_new = 0.5 * m_shaft_new * r_inner_new^2;
I_mag_new = 0.5 * m_mag_new * (r_mag_new^2 + r_inner_new^2);
I_total_new = I_flywheel_new + I_shaft_new + I_mag_new;

m_eff_tilt_new = I_total_new / L_amb_new^2;

for i = 1:length(omega)
    s = 1j * omega(i);

    % Radial stiffness
    stiff_baseline_radial(i) = abs(baseline.total_mass * s^2 + c_eff_baseline * s + k_eff_baseline);
    stiff_new_radial(i) = abs(optimal.total_mass * s^2 + c_eff_new * s + k_eff_new);

    % Tilting stiffness (two AMBs)
    stiff_baseline_tilt(i) = abs(2 * (m_eff_tilt_baseline * s^2 + c_eff_baseline * s + k_eff_baseline));
    stiff_new_tilt(i) = abs(2 * (m_eff_tilt_new * s^2 + c_eff_new * s + k_eff_new));
end

% Plot dynamic stiffness comparison
figure('Name', 'Dynamic Stiffness Comparison', 'Position', [100 100 1000 500]);

subplot(1,2,1);
loglog(freq, stiff_baseline_radial/1e6, 'b-', 'LineWidth', 2); hold on;
loglog(freq, stiff_new_radial/1e6, 'r--', 'LineWidth', 2);
grid on;
xlabel('Frequency [Hz]', 'FontSize', 11);
ylabel('Dynamic Stiffness [MN/m]', 'FontSize', 11);
title('Radial Dynamic Stiffness', 'FontSize', 12);
legend('Baseline', 'New Design', 'Location', 'best');

subplot(1,2,2);
loglog(freq, stiff_baseline_tilt/1e6, 'b-', 'LineWidth', 2); hold on;
loglog(freq, stiff_new_tilt/1e6, 'r--', 'LineWidth', 2);
grid on;
xlabel('Frequency [Hz]', 'FontSize', 11);
ylabel('Dynamic Stiffness [MN/m]', 'FontSize', 11);
title('Tilting Dynamic Stiffness', 'FontSize', 12);
legend('Baseline', 'New Design', 'Location', 'best');

saveas(gcf, 'part3c_dynamic_stiffness.fig');
saveas(gcf, 'part3c_dynamic_stiffness.png');

fprintf('Dynamic Stiffness at Key Frequencies:\n');
fprintf('                          Baseline      New Design\n');
fprintf('  Radial at 10 Hz:        %.2f MN/m    %.2f MN/m\n', ...
    interp1(freq, stiff_baseline_radial, 10)/1e6, interp1(freq, stiff_new_radial, 10)/1e6);
fprintf('  Radial at 100 Hz:       %.2f MN/m   %.2f MN/m\n', ...
    interp1(freq, stiff_baseline_radial, 100)/1e6, interp1(freq, stiff_new_radial, 100)/1e6);

%% ========================================================================
% SECTION 8: ROTOR RUNOUT
% ========================================================================

fprintf('\n==============================================\n');
fprintf('PART 3c: Rotor Runout Comparison\n');
fprintf('==============================================\n\n');

% ISO G2.5 balance grade
G_grade = 2.5;  % mm/s

% SoC range
SoC = linspace(0, 100, 100);

omega_baseline = baseline.max_speed_rpm * 2*pi/60 * (0.5 + 0.5*SoC/100);
omega_new = optimal.max_speed_rpm * 2*pi/60 * (0.5 + 0.5*SoC/100);

runout_baseline = zeros(size(SoC));
runout_new = zeros(size(SoC));

for i = 1:length(SoC)
    % Baseline
    e_bl = (G_grade * 1e-3) / omega_baseline(i);
    F_unb_bl = baseline.total_mass * e_bl * omega_baseline(i)^2;
    denom_bl = sqrt((k_eff_baseline - baseline.total_mass*omega_baseline(i)^2)^2 + ...
        (c_eff_baseline*omega_baseline(i))^2);
    runout_baseline(i) = F_unb_bl / denom_bl * 1e6;

    % New design
    e_new = (G_grade * 1e-3) / omega_new(i);
    F_unb_new = optimal.total_mass * e_new * omega_new(i)^2;
    denom_new = sqrt((k_eff_new - optimal.total_mass*omega_new(i)^2)^2 + ...
        (c_eff_new*omega_new(i))^2);
    runout_new(i) = F_unb_new / denom_new * 1e6;
end

% Plot runout comparison
figure('Name', 'Rotor Runout Comparison', 'Position', [100 100 800 500]);

plot(SoC, runout_baseline, 'b-', 'LineWidth', 2.5); hold on;
plot(SoC, runout_new, 'r--', 'LineWidth', 2.5);
grid on;
xlabel('State of Charge [%]', 'FontSize', 12);
ylabel('Rotor Runout [\mum]', 'FontSize', 12);
title('Rotor Runout Due to Mass Imbalance (ISO G2.5)', 'FontSize', 14);
legend('Baseline', 'New Design', 'Location', 'best', 'FontSize', 12);

saveas(gcf, 'part3c_rotor_runout.fig');
saveas(gcf, 'part3c_rotor_runout.png');

fprintf('Rotor Runout (ISO G2.5 balance grade):\n');
fprintf('                          Baseline      New Design\n');
fprintf('  At 0%% SoC:              %.2f µm       %.2f µm\n', runout_baseline(1), runout_new(1));
fprintf('  At 50%% SoC:             %.2f µm       %.2f µm\n', runout_baseline(50), runout_new(50));
fprintf('  At 100%% SoC:            %.2f µm       %.2f µm\n', runout_baseline(end), runout_new(end));

%% ========================================================================
% SECTION 9: SUMMARY
% ========================================================================

fprintf('\n==============================================\n');
fprintf('DELIVERABLE 3 SUMMARY\n');
fprintf('==============================================\n\n');

fprintf('CONTROLLER TRANSFER FUNCTIONS:\n\n');
fprintf('1. Current Controller (both systems):\n');
fprintf('   G_ci(s) = (258.75*s + 1611.54) / s\n\n');

fprintf('2. Position Controller - Baseline:\n');
fprintf('   G_pos(s) = %.4f*s + %.2f\n', Kd_pos_baseline, Kp_pos_baseline);
fprintf('   Natural freq: %.1f Hz, Damping: %.2f\n\n', omega_n_cl_baseline/(2*pi), zeta_cl_baseline);

fprintf('3. Position Controller - New Design:\n');
fprintf('   G_pos(s) = %.4f*s + %.2f\n', Kd_pos_new, Kp_pos_new);
fprintf('   Natural freq: %.1f Hz, Damping: %.2f\n\n', omega_n_cl_new/(2*pi), zeta_cl_new);

fprintf('PERFORMANCE COMPARISON:\n\n');
fprintf('                          Baseline      New Design    Assessment\n');
fprintf('---------------------------------------------------------------\n');
fprintf('Peak step displacement:   %.2f µm      %.2f µm      %s\n', ...
    peak_disp_baseline, peak_disp_new, iif(peak_disp_new <= peak_disp_baseline, 'IMPROVED', 'DEGRADED'));
fprintf('Settling time:            %.1f ms       %.1f ms       %s\n', ...
    settling_time_baseline, settling_time_new, iif(settling_time_new <= settling_time_baseline, 'IMPROVED', 'COMPARABLE'));
fprintf('Stiffness at 10 Hz:       %.2f MN/m    %.2f MN/m    %s\n', ...
    interp1(freq, stiff_baseline_radial, 10)/1e6, interp1(freq, stiff_new_radial, 10)/1e6, ...
    iif(interp1(freq, stiff_new_radial, 10) >= interp1(freq, stiff_baseline_radial, 10), 'IMPROVED', 'COMPARABLE'));
fprintf('Max runout:               %.2f µm       %.2f µm       %s\n', ...
    max(runout_baseline), max(runout_new), iif(max(runout_new) <= max(runout_baseline), 'IMPROVED', 'COMPARABLE'));
fprintf('---------------------------------------------------------------\n');

fprintf('\nAll plots saved to current folder.\n');
fprintf('==============================================\n');

function r = iif(cond, a, b)
    if cond
        r = a;
    else
        r = b;
    end
end
