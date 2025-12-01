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
% Controller Structure (from Appendix B):
%   Current controller: G_ci(s) = 345 + 2149/s
%   Position controller: G_pos(s) = kp + ki/s + s*kd/(1+s/wp)
%
% Author: Team 16
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
% SECTION 2: BASELINE SYSTEM PARAMETERS (FROM APPENDIX B)
% ========================================================================

fprintf('Loading baseline system parameters...\n');

% Baseline parameters (from Appendix B - Table A.1)
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
% SECTION 3: BASELINE CONTROLLER PARAMETERS (FROM APPENDIX B)
% ========================================================================

fprintf('\n==============================================\n');
fprintf('BASELINE CONTROLLER PARAMETERS (Appendix B)\n');
fprintf('==============================================\n\n');

% Current controller transfer function (from Appendix B)
% G_ci(s) = v(s)/e(s) = 345 + 2149/s
Kp_current = 345;
Ki_current = 2149;

fprintf('Current Controller (given in Appendix B):\n');
fprintf('  G_ci(s) = Kp + Ki/s = %.0f + %.0f/s\n', Kp_current, Ki_current);
fprintf('  Transfer function: G_ci(s) = (%.0f*s + %.0f) / s\n\n', Kp_current, Ki_current);

% Position Controller X (PID with filtered derivative) - from Appendix B
% G_pos_x(s) = kpx + kix/s + s*kdx/(1 + s/wpx)
baseline.kpx = 1.2639e8;       % Proportional gain [N/m]
baseline.kix = 1.16868e9;      % Integral gain [N/(m*s)]
baseline.kdx = 252790;         % Derivative gain [N*s/m]
baseline.omega_px = 3770;      % Derivative filter cutoff [rad/s]

fprintf('Position Controller X (PID with filtered derivative):\n');
fprintf('  G_pos_x(s) = kpx + kix/s + s*kdx/(1 + s/wpx)\n');
fprintf('  kpx = %.4e N/m\n', baseline.kpx);
fprintf('  kix = %.5e N/(m*s)\n', baseline.kix);
fprintf('  kdx = %.0f N*s/m\n', baseline.kdx);
fprintf('  wpx = %.0f rad/s (fp = %.0f Hz)\n\n', baseline.omega_px, baseline.omega_px/(2*pi));

% Tilting Position Controller (PID with filtered derivative) - from Appendix B
% G_pos_alpha(s) = kp_alpha + ki_alpha/s + s*kd_alpha/(1 + s/wp_alpha)
baseline.kp_alpha = 7.6992e7;    % Proportional gain [N*m/rad]
baseline.ki_alpha = 1.18953e9;   % Integral gain [N*m/(rad*s)]
baseline.kd_alpha = 80294;       % Derivative gain [N*m*s/rad]
baseline.omega_p_alpha = 6283;   % Derivative filter cutoff [rad/s]

fprintf('Tilting Position Controller:\n');
fprintf('  G_pos_alpha(s) = kp_alpha + ki_alpha/s + s*kd_alpha/(1 + s/wp_alpha)\n');
fprintf('  kp_alpha = %.4e N*m/rad\n', baseline.kp_alpha);
fprintf('  ki_alpha = %.5e N*m/(rad*s)\n', baseline.ki_alpha);
fprintf('  kd_alpha = %.0f N*m*s/rad\n', baseline.kd_alpha);
fprintf('  wp_alpha = %.0f rad/s (fp = %.0f Hz)\n\n', baseline.omega_p_alpha, baseline.omega_p_alpha/(2*pi));

%% ========================================================================
% SECTION 4: NEW SYSTEM AMB SIZING
% ========================================================================

fprintf('==============================================\n');
fprintf('NEW SYSTEM AMB SIZING\n');
fprintf('==============================================\n\n');

% AMB rated force = 2 x rotating group weight (per project requirements)
g = 9.81;  % Gravity [m/s^2]
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

% Extract AMB physical parameters
K_s_baseline = amb_baseline.stiffnessConstant;  % Negative stiffness magnitude [N/m]
K_i_baseline = amb_baseline.forceConstant;       % Force constant [N/A]

K_s_new = amb_new.stiffnessConstant;
K_i_new = amb_new.forceConstant;

%% ========================================================================
% SECTION 5: NEW SYSTEM CONTROLLER DESIGN (HW3 METHODOLOGY)
% ========================================================================

fprintf('\n==============================================\n');
fprintf('PART 3a: NEW SYSTEM CONTROLLER DESIGN\n');
fprintf('==============================================\n\n');

% Design approach: Use HW3 methodology for controller design from first principles
% - Current controller: Pole-zero cancellation for first-order closed-loop
% - Position controller: Target specific crossover frequency

% Calculate new inertia (still needed for tilting controller)
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

% AMB separation distances
L_amb_baseline = baseline.flywheel_length + 0.3;
L_amb_new = optimal.flywheel_length + 0.3;

% ----- CURRENT CONTROLLER (HW3 Q2a methodology) -----
% Plant: G_p(s) = 1/(L*s + R)
% Design PI controller for first-order closed-loop with 1.5 kHz bandwidth
% Pole-zero cancellation: ki/kp = R/L (zero cancels plant pole)
% Closed-loop bandwidth: kp = L * omega_bw

fprintf('CURRENT CONTROLLER DESIGN (HW3 Q2a methodology):\n');
fprintf('  Design method: Pole-zero cancellation for first-order response\n');
fprintf('  Target bandwidth: 1.5 kHz\n\n');

omega_bw_current = 2*pi*1500;  % 1.5 kHz bandwidth target

L_coil_new = amb_new.coilInductance;
R_coil_new = amb_new.coilResistance;

Kp_current_new = L_coil_new * omega_bw_current;
Ki_current_new = Kp_current_new * R_coil_new / L_coil_new;

fprintf('  New System Coil Parameters:\n');
fprintf('    Inductance L = %.4f H\n', L_coil_new);
fprintf('    Resistance R = %.3f Ohms\n', R_coil_new);
fprintf('  New Current Controller G_ci(s) = kp + ki/s:\n');
fprintf('    kp = %.2f\n', Kp_current_new);
fprintf('    ki = %.2f\n', Ki_current_new);
fprintf('  Closed-loop time constant: %.4f ms\n\n', 1000*L_coil_new/Kp_current_new);

% ----- POSITION CONTROLLER (HW3 Q2e methodology) -----
% Plant (assuming fast current loop): Y(s)/I(s) = Ki / (m*s^2 - |Ks|)
% PID controller: G_cp(s) = kp + ki/s + s*kd/(1 + s/omega_p)

fprintf('POSITION CONTROLLER DESIGN (HW3 Q2e methodology):\n');

% Unstable pole frequency (HW3 Q2d)
omega_unstable_new = sqrt(abs(K_s_new) / optimal.total_mass);
f_unstable_new = omega_unstable_new / (2*pi);
fprintf('  Unstable pole frequency: %.2f Hz\n', f_unstable_new);

% Target crossover frequency (from HW3: 100 Hz)
f_crossover = 100;  % Hz
omega_crossover = 2*pi*f_crossover;
fprintf('  Target crossover frequency: %.0f Hz\n', f_crossover);

% Derivative filter at 10x crossover (standard practice from HW3)
new.omega_px = 10 * omega_crossover;
fprintf('  Derivative filter cutoff: %.0f Hz (10x crossover)\n\n', new.omega_px/(2*pi));

% Controller gains designed for stability and disturbance rejection
% At crossover: |L(jw)| = |G_cp * Ki / (m*w^2 + |Ks|)| = 1
plant_mag_at_crossover = K_i_new / abs(-optimal.total_mass * omega_crossover^2 - K_s_new);

% Proportional gain: Set for unity gain at crossover
new.kpx = 1 / plant_mag_at_crossover;

% Integral gain: Integral corner at crossover/10 for steady-state accuracy
omega_int = omega_crossover / 10;
new.kix = new.kpx * omega_int;

% Derivative gain: Derivative corner at crossover for phase lead
new.kdx = new.kpx / omega_crossover;

fprintf('  Position Controller X: G_pos(s) = kp + ki/s + s*kd/(1 + s/wp)\n');
fprintf('    kp = %.4e\n', new.kpx);
fprintf('    ki = %.4e\n', new.kix);
fprintf('    kd = %.4e\n', new.kdx);
fprintf('    wp = %.0f rad/s (%.0f Hz)\n\n', new.omega_px, new.omega_px/(2*pi));

% ----- TILTING CONTROLLER (scaled from radial using HW3 principles) -----
% For tilting mode, effective mass is I/L^2 and use same crossover target
m_eff_tilt_new = I_total_new / (L_amb_new/2)^2;
K_s_tilt_new = K_s_new * 2;  % Two AMBs contribute
K_i_tilt_new = K_i_new * 2 * (L_amb_new/2);  % Torque = 2 * F * arm

plant_mag_tilt = K_i_tilt_new / abs(-m_eff_tilt_new * omega_crossover^2 - K_s_tilt_new);

new.kp_alpha = 1 / plant_mag_tilt;
new.ki_alpha = new.kp_alpha * omega_int;
new.kd_alpha = new.kp_alpha / omega_crossover;
new.omega_p_alpha = new.omega_px;

fprintf('  Tilting Controller: G_pos_alpha(s) = kp + ki/s + s*kd/(1 + s/wp)\n');
fprintf('    kp_alpha = %.4e\n', new.kp_alpha);
fprintf('    ki_alpha = %.4e\n', new.ki_alpha);
fprintf('    kd_alpha = %.4e\n', new.kd_alpha);
fprintf('    wp_alpha = %.0f rad/s\n\n', new.omega_p_alpha);

% ----- STABILITY MARGIN VERIFICATION -----
fprintf('STABILITY MARGIN VERIFICATION:\n');

s_tf = tf('s');

% Position controller transfer function
G_pos_new_tf = new.kpx + new.kix/s_tf + new.kdx*s_tf/(1 + s_tf/new.omega_px);

% Plant (AMB + mass) - Note: K_s_new is positive magnitude of negative stiffness
G_plant_new_tf = K_i_new / (optimal.total_mass*s_tf^2 - K_s_new);

% Open-loop transfer function
L_new = G_pos_new_tf * G_plant_new_tf;

% Calculate margins
[Gm, Pm, Wcg, Wcp] = margin(L_new);

fprintf('  Gain margin: %.1f dB at %.1f Hz\n', 20*log10(Gm), Wcg/(2*pi));
fprintf('  Phase margin: %.1f deg at %.1f Hz\n', Pm, Wcp/(2*pi));

if Pm > 30 && Gm > 2
    fprintf('  Status: STABLE (adequate margins)\n\n');
else
    fprintf('  Status: REVIEW (low margins)\n\n');
end

% ----- DISTURBANCE REJECTION CHECK (HW3 Q2f) -----
fprintf('DISTURBANCE REJECTION CHECK:\n');

f_dist = 10;  % Hz (from HW3 spec)
F_dist = 5;   % N (from HW3 spec)
s_dist = 1j * 2*pi*f_dist;

G_pos_at_dist = new.kpx + new.kix/s_dist + new.kdx*s_dist/(1 + s_dist/new.omega_px);
denom_at_dist = optimal.total_mass*s_dist^2 - K_s_new + G_pos_at_dist*K_i_new;
X_dist = F_dist / abs(denom_at_dist);

fprintf('  Disturbance: %.0f N at %.0f Hz\n', F_dist, f_dist);
fprintf('  Deflection: %.4f mm\n', X_dist*1000);
fprintf('  Requirement: < 0.5 mm\n');
fprintf('  Status: %s\n\n', iif(X_dist*1000 < 0.5, 'PASS', 'FAIL'));

%% ========================================================================
% SECTION 6: TRANSFER FUNCTION SUMMARY
% ========================================================================

fprintf('==============================================\n');
fprintf('PART 3a: COMPLETE TRANSFER FUNCTIONS\n');
fprintf('==============================================\n\n');

fprintf('AMB SYSTEM TRANSFER FUNCTIONS\n\n');

fprintf('1. Current Controller - Baseline (from Appendix B):\n');
fprintf('   G_ci(s) = %.0f + %.0f/s\n\n', Kp_current, Ki_current);

fprintf('2. Current Controller - New Design (HW3 methodology, 1.5 kHz BW):\n');
fprintf('   G_ci(s) = %.2f + %.2f/s\n\n', Kp_current_new, Ki_current_new);

fprintf('3. Position Controller X - Baseline:\n');
fprintf('   G_pos_x(s) = %.4e + %.5e/s + s*%.0f/(1 + s/%.0f)\n', ...
    baseline.kpx, baseline.kix, baseline.kdx, baseline.omega_px);
fprintf('   Simplified form:\n');
fprintf('   G_pos_x(s) = [%.4e*s*(1+s/%.0f) + %.5e*(1+s/%.0f) + s^2*%.0f] / [s*(1+s/%.0f)]\n\n', ...
    baseline.kpx, baseline.omega_px, baseline.kix, baseline.omega_px, baseline.kdx, baseline.omega_px);

fprintf('4. Position Controller X - New Design (HW3 methodology, 100 Hz crossover):\n');
fprintf('   G_pos_x(s) = %.4e + %.4e/s + s*%.4e/(1 + s/%.0f)\n\n', ...
    new.kpx, new.kix, new.kdx, new.omega_px);

fprintf('5. Tilting Controller - Baseline:\n');
fprintf('   G_pos_alpha(s) = %.4e + %.5e/s + s*%.0f/(1 + s/%.0f)\n\n', ...
    baseline.kp_alpha, baseline.ki_alpha, baseline.kd_alpha, baseline.omega_p_alpha);

fprintf('6. Tilting Controller - New Design (HW3 methodology):\n');
fprintf('   G_pos_alpha(s) = %.4e + %.4e/s + s*%.4e/(1 + s/%.0f)\n\n', ...
    new.kp_alpha, new.ki_alpha, new.kd_alpha, new.omega_p_alpha);

fprintf('7. Plant Model (AMB + Rotor):\n');
fprintf('   G_plant(s) = Ki / (m*s^2 + Ks)  [Note: Ks is negative stiffness]\n');
fprintf('   Baseline: G_plant(s) = %.2f / (%.2f*s^2 - %.2e)\n', ...
    K_i_baseline, baseline.total_mass, K_s_baseline);
fprintf('   New:      G_plant(s) = %.2f / (%.2f*s^2 - %.2e)\n\n', ...
    K_i_new, optimal.total_mass, K_s_new);

%% ========================================================================
% SECTION 7: STEP RESPONSE SIMULATION
% ========================================================================

fprintf('==============================================\n');
fprintf('PART 3b: AMB STEP RESPONSE COMPARISON\n');
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
x_baseline = zeros(size(t_sim));
x_new = zeros(size(t_sim));
i_baseline = zeros(size(t_sim));
i_new = zeros(size(t_sim));
f_baseline = zeros(size(t_sim));
f_new = zeros(size(t_sim));

% State variables for simulation: x (position), v (velocity), i (current), xi (integral of position error)
% BASELINE simulation at 0 RPM
x = 0; v = 0; i = 0; xi = 0;
i_filt = 0;  % Filtered derivative term

% Current controller time constant
tau_i = amb_baseline.coilInductance / amb_baseline.coilResistance;

for k = 1:length(t_sim)
    % Position error (reference = 0)
    e_pos = 0 - x;

    % PID controller with filtered derivative
    % G_pos(s) = kp + ki/s + kd*s/(1 + s/wp)
    % Proportional term
    i_p = baseline.kpx * e_pos;

    % Integral term (accumulated error)
    xi = xi + e_pos * dt;
    i_i = baseline.kix * xi;

    % Filtered derivative term (using first-order filter on velocity)
    % d/dt(i_filt) = wp * (kd * (-v) - i_filt)
    di_filt = baseline.omega_px * (baseline.kdx * (-v) - i_filt);
    i_filt = i_filt + di_filt * dt;
    i_d = i_filt;

    % Total current command
    i_cmd = (i_p + i_i + i_d) / K_i_baseline;  % Convert force command to current

    % Current controller dynamics (simplified first-order response)
    di_dt = (i_cmd - i) * Kp_current + Ki_current * (i_cmd - i) * dt;
    i = i + di_dt * dt;

    % AMB force: F = Ki*i - Ks*x (Ks is negative stiffness, so -Ks*x is destabilizing)
    F_amb = K_i_baseline * i - K_s_baseline * x;

    % Equation of motion: m*a = F_amb + F_disturbance
    a = (F_amb + F_step_baseline) / baseline.total_mass;

    % Integrate
    v = v + a * dt;
    x = x + v * dt;

    % Store
    x_baseline(k) = x;
    i_baseline(k) = i;
    f_baseline(k) = F_amb;
end

% NEW SYSTEM simulation at 0 RPM
x = 0; v = 0; i = 0; xi = 0;
i_filt = 0;

tau_i_new = amb_new.coilInductance / amb_new.coilResistance;

for k = 1:length(t_sim)
    % Position error
    e_pos = 0 - x;

    % PID controller with filtered derivative
    i_p = new.kpx * e_pos;

    xi = xi + e_pos * dt;
    i_i = new.kix * xi;

    di_filt = new.omega_px * (new.kdx * (-v) - i_filt);
    i_filt = i_filt + di_filt * dt;
    i_d = i_filt;

    i_cmd = (i_p + i_i + i_d) / K_i_new;

    % Use new current controller (HW3 methodology)
    di_dt = (i_cmd - i) * Kp_current_new + Ki_current_new * (i_cmd - i) * dt;
    i = i + di_dt * dt;

    F_amb = K_i_new * i - K_s_new * x;

    a = (F_amb + F_step_new) / optimal.total_mass;

    v = v + a * dt;
    x = x + v * dt;

    x_new(k) = x;
    i_new(k) = i;
    f_new(k) = F_amb;
end

% Create comparison plots
figure('Name', 'AMB Step Response Comparison', 'Position', [100 100 1200 800]);

% Position response
subplot(2,3,1);
plot(t_sim*1000, x_baseline*1e6, 'b-', 'LineWidth', 2); hold on;
plot(t_sim*1000, x_new*1e6, 'r--', 'LineWidth', 2);
grid on;
xlabel('Time [ms]', 'FontSize', 11);
ylabel('Position [\mum]', 'FontSize', 11);
title('Position Response at 0 RPM', 'FontSize', 12);
legend('Baseline', 'New Design', 'Location', 'best');

% Current response
subplot(2,3,2);
plot(t_sim*1000, i_baseline, 'b-', 'LineWidth', 2); hold on;
plot(t_sim*1000, i_new, 'r--', 'LineWidth', 2);
grid on;
xlabel('Time [ms]', 'FontSize', 11);
ylabel('Current [A]', 'FontSize', 11);
title('Current Response at 0 RPM', 'FontSize', 12);
legend('Baseline', 'New Design', 'Location', 'best');

% Force response
subplot(2,3,3);
plot(t_sim*1000, f_baseline, 'b-', 'LineWidth', 2); hold on;
plot(t_sim*1000, f_new, 'r--', 'LineWidth', 2);
grid on;
xlabel('Time [ms]', 'FontSize', 11);
ylabel('AMB Force [N]', 'FontSize', 11);
title('Force Response at 0 RPM', 'FontSize', 12);
legend('Baseline', 'New Design', 'Location', 'best');

% Position time history (zoomed)
subplot(2,3,4);
plot(t_sim*1000, x_baseline*1e6, 'b-', 'LineWidth', 2); hold on;
plot(t_sim*1000, x_new*1e6, 'r--', 'LineWidth', 2);
grid on;
xlim([0 10]);
xlabel('Time [ms]', 'FontSize', 11);
ylabel('Position [\mum]', 'FontSize', 11);
title('Position (First 10 ms)', 'FontSize', 12);
legend('Baseline', 'New Design', 'Location', 'best');

% Steady-state position comparison
subplot(2,3,5);
ss_start = round(0.8*length(t_sim));
bar_data = [mean(x_baseline(ss_start:end))*1e6, mean(x_new(ss_start:end))*1e6];
bar(bar_data);
set(gca, 'XTickLabel', {'Baseline', 'New Design'});
ylabel('Steady-State Position [\mum]', 'FontSize', 11);
title('Steady-State Comparison', 'FontSize', 12);
grid on;

% Performance metrics text
subplot(2,3,6);
axis off;

% Calculate metrics
peak_baseline = max(abs(x_baseline)) * 1e6;
peak_new = max(abs(x_new)) * 1e6;

% Settling time (2% criterion)
ss_value_bl = x_baseline(end);
settling_idx_bl = find(abs(x_baseline - ss_value_bl) > 0.02*peak_baseline, 1, 'last');
if isempty(settling_idx_bl), settling_idx_bl = 1; end
settling_time_bl = t_sim(settling_idx_bl) * 1000;

ss_value_new = x_new(end);
settling_idx_new = find(abs(x_new - ss_value_new) > 0.02*peak_new, 1, 'last');
if isempty(settling_idx_new), settling_idx_new = 1; end
settling_time_new = t_sim(settling_idx_new) * 1000;

text(0.1, 0.9, 'STEP RESPONSE METRICS', 'FontSize', 14, 'FontWeight', 'bold');
text(0.1, 0.75, sprintf('Peak Displacement:'), 'FontSize', 11, 'FontWeight', 'bold');
text(0.1, 0.65, sprintf('  Baseline: %.3f um', peak_baseline), 'FontSize', 11);
text(0.1, 0.55, sprintf('  New Design: %.3f um', peak_new), 'FontSize', 11);
text(0.1, 0.40, sprintf('Settling Time (2%%):'), 'FontSize', 11, 'FontWeight', 'bold');
text(0.1, 0.30, sprintf('  Baseline: %.2f ms', settling_time_bl), 'FontSize', 11);
text(0.1, 0.20, sprintf('  New Design: %.2f ms', settling_time_new), 'FontSize', 11);

saveas(gcf, 'part3b_step_response.fig');
saveas(gcf, 'part3b_step_response.png');

fprintf('\nStep Response Performance:\n');
fprintf('                          Baseline      New Design\n');
fprintf('  Peak displacement:      %.3f um      %.3f um\n', peak_baseline, peak_new);
fprintf('  Settling time (2%%):     %.2f ms       %.2f ms\n', settling_time_bl, settling_time_new);

%% ========================================================================
% SECTION 8: DYNAMIC STIFFNESS
% ========================================================================

fprintf('\n==============================================\n');
fprintf('PART 3c: Dynamic Stiffness Comparison\n');
fprintf('==============================================\n\n');

% Frequency range
freq = logspace(0, 3, 200);  % 1-1000 Hz
omega = 2*pi*freq;

% Calculate dynamic stiffness for both systems
% Dynamic stiffness = |F/X| from closed-loop transfer function
% For PID controller: G_pos(s) = kp + ki/s + kd*s/(1+s/wp)
% Closed-loop: X/F_dist = 1 / (m*s^2 + G_pos(s)*Ki - Ks)

stiff_baseline_radial = zeros(size(omega));
stiff_new_radial = zeros(size(omega));
stiff_baseline_tilt = zeros(size(omega));
stiff_new_tilt = zeros(size(omega));

for i = 1:length(omega)
    s = 1j * omega(i);

    % Baseline radial stiffness
    G_pos_bl = baseline.kpx + baseline.kix/s + baseline.kdx*s/(1 + s/baseline.omega_px);
    denom_bl = baseline.total_mass * s^2 + G_pos_bl - K_s_baseline;
    stiff_baseline_radial(i) = abs(denom_bl);

    % New system radial stiffness
    G_pos_new = new.kpx + new.kix/s + new.kdx*s/(1 + s/new.omega_px);
    denom_new = optimal.total_mass * s^2 + G_pos_new - K_s_new;
    stiff_new_radial(i) = abs(denom_new);

    % Tilting stiffness (two AMBs, effective mass = I/L^2)
    m_eff_tilt_bl = baseline.I_total / L_amb_baseline^2;
    G_tilt_bl = baseline.kp_alpha + baseline.ki_alpha/s + baseline.kd_alpha*s/(1 + s/baseline.omega_p_alpha);
    denom_tilt_bl = m_eff_tilt_bl * s^2 + 2*G_tilt_bl - 2*K_s_baseline;
    stiff_baseline_tilt(i) = abs(denom_tilt_bl);

    m_eff_tilt_new = I_total_new / L_amb_new^2;
    G_tilt_new = new.kp_alpha + new.ki_alpha/s + new.kd_alpha*s/(1 + s/new.omega_p_alpha);
    denom_tilt_new = m_eff_tilt_new * s^2 + 2*G_tilt_new - 2*K_s_new;
    stiff_new_tilt(i) = abs(denom_tilt_new);
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
fprintf('  Radial at 500 Hz:       %.2f MN/m   %.2f MN/m\n', ...
    interp1(freq, stiff_baseline_radial, 500)/1e6, interp1(freq, stiff_new_radial, 500)/1e6);

%% ========================================================================
% SECTION 9: ROTOR RUNOUT
% ========================================================================

fprintf('\n==============================================\n');
fprintf('PART 3c: Rotor Runout Comparison\n');
fprintf('==============================================\n\n');

% ISO G2.5 balance grade
G_grade = 2.5;  % mm/s

% SoC range
SoC = linspace(0, 100, 100);

% Speed arrays
omega_baseline_arr = baseline.max_speed_rpm * 2*pi/60 * (0.5 + 0.5*SoC/100);
omega_new_arr = optimal.max_speed_rpm * 2*pi/60 * (0.5 + 0.5*SoC/100);

runout_baseline = zeros(size(SoC));
runout_new = zeros(size(SoC));

for i = 1:length(SoC)
    w_bl = omega_baseline_arr(i);
    w_new = omega_new_arr(i);

    % Eccentricity from balance grade
    e_bl = (G_grade * 1e-3) / w_bl;
    e_new = (G_grade * 1e-3) / w_new;

    % Unbalance force
    F_unb_bl = baseline.total_mass * e_bl * w_bl^2;
    F_unb_new = optimal.total_mass * e_new * w_new^2;

    % Dynamic stiffness at synchronous frequency
    s_bl = 1j * w_bl;
    s_new = 1j * w_new;

    G_pos_bl = baseline.kpx + baseline.kix/s_bl + baseline.kdx*s_bl/(1 + s_bl/baseline.omega_px);
    denom_bl = baseline.total_mass * s_bl^2 + G_pos_bl - K_s_baseline;

    G_pos_new = new.kpx + new.kix/s_new + new.kdx*s_new/(1 + s_new/new.omega_px);
    denom_new = optimal.total_mass * s_new^2 + G_pos_new - K_s_new;

    % Runout = F_unb / |dynamic stiffness|
    runout_baseline(i) = F_unb_bl / abs(denom_bl) * 1e6;  % [um]
    runout_new(i) = F_unb_new / abs(denom_new) * 1e6;  % [um]
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
fprintf('  At 0%% SoC:              %.2f um       %.2f um\n', runout_baseline(1), runout_new(1));
fprintf('  At 50%% SoC:             %.2f um       %.2f um\n', runout_baseline(50), runout_new(50));
fprintf('  At 100%% SoC:            %.2f um       %.2f um\n', runout_baseline(end), runout_new(end));

%% ========================================================================
% SECTION 10: SUMMARY
% ========================================================================

fprintf('\n==============================================\n');
fprintf('DELIVERABLE 3 SUMMARY\n');
fprintf('==============================================\n\n');

fprintf('CONTROLLER TRANSFER FUNCTIONS:\n\n');
fprintf('1. Current Controller - Baseline (from Appendix B):\n');
fprintf('   G_ci(s) = %.0f + %.0f/s\n\n', Kp_current, Ki_current);

fprintf('2. Current Controller - New Design (HW3 methodology, 1.5 kHz BW):\n');
fprintf('   G_ci(s) = %.2f + %.2f/s\n\n', Kp_current_new, Ki_current_new);

fprintf('3. Position Controller X - Baseline (from Appendix B):\n');
fprintf('   G_pos(s) = kp + ki/s + kd*s/(1+s/wp)\n');
fprintf('   kp = %.4e, ki = %.5e, kd = %.0f, wp = %.0f rad/s\n\n', ...
    baseline.kpx, baseline.kix, baseline.kdx, baseline.omega_px);

fprintf('4. Position Controller X - New Design (HW3 methodology, 100 Hz crossover):\n');
fprintf('   kp = %.4e, ki = %.4e, kd = %.0f, wp = %.0f rad/s\n\n', ...
    new.kpx, new.kix, new.kdx, new.omega_px);

fprintf('5. Tilting Controller - Baseline:\n');
fprintf('   kp = %.4e, ki = %.5e, kd = %.0f, wp = %.0f rad/s\n\n', ...
    baseline.kp_alpha, baseline.ki_alpha, baseline.kd_alpha, baseline.omega_p_alpha);

fprintf('6. Tilting Controller - New Design (HW3 methodology):\n');
fprintf('   kp = %.4e, ki = %.4e, kd = %.0f, wp = %.0f rad/s\n\n', ...
    new.kp_alpha, new.ki_alpha, new.kd_alpha, new.omega_p_alpha);

fprintf('PERFORMANCE COMPARISON:\n\n');
fprintf('                          Baseline      New Design    Assessment\n');
fprintf('---------------------------------------------------------------\n');
fprintf('Peak step displacement:   %.3f um      %.3f um      %s\n', ...
    peak_baseline, peak_new, iif(peak_new <= peak_baseline*1.1, 'COMPARABLE', 'REVIEW'));
fprintf('Settling time:            %.2f ms       %.2f ms       %s\n', ...
    settling_time_bl, settling_time_new, iif(settling_time_new <= settling_time_bl*1.2, 'COMPARABLE', 'REVIEW'));
fprintf('Stiffness at 10 Hz:       %.2f MN/m    %.2f MN/m    %s\n', ...
    interp1(freq, stiff_baseline_radial, 10)/1e6, interp1(freq, stiff_new_radial, 10)/1e6, ...
    iif(interp1(freq, stiff_new_radial, 10) >= 0.8*interp1(freq, stiff_baseline_radial, 10), 'COMPARABLE', 'REVIEW'));
fprintf('Max runout:               %.2f um       %.2f um       %s\n', ...
    max(runout_baseline), max(runout_new), iif(max(runout_new) <= max(runout_baseline)*1.5, 'COMPARABLE', 'REVIEW'));
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
