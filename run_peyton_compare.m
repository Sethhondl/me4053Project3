%% Run Peyton's code and save plots for comparison
% This script runs Peyton's Deliverable 1 analysis and saves the plots

clc; clear; close all;

% Add functions path
addpath('Project3_Functions');

%% Run Peyton's analysis (copy the relevant functions inline)

params = createParams();
format = createPlotFormatStruct();

% Run Deliverable 1 analysis
runDeliverable1_Peyton(params, format);

%% Local functions (copied from peytonVersion.m)

function params = createParams()
    params = struct();

    % Thermal / environment
    params.deliverable1.thermal.housingTemp_C     = 30;
    params.deliverable1.thermal.maxTemp_C         = 100;
    params.deliverable1.thermal.rotorEmissivity   = 0.4;
    params.deliverable1.thermal.housingEmissivity = 0.9;

    % Geometry
    params.deliverable1.geometry.flywheelAxialLength_m = 1000e-3;
    params.deliverable1.geometry.flywheelDiameter_m    = 430e-3;
    params.deliverable1.geometry.motorAxialLength_m    = 250e-3;
    params.deliverable1.geometry.shaftAndPMDiameter_m  = 84e-3;
    params.deliverable1.geometry.magnetThickness_m     = 6e-3;
    params.deliverable1.geometry.minMagnetThickness_m  = 2e-3;

    % AMB rated force
    params.deliverable1.amb.ratedForce_N = 5780;

    % Speed limits
    params.deliverable1.speed.maxRotSpeed_rpm       = 40000;
    params.deliverable1.speed.maxRotSpeed_radPerSec = 40000 * 2*pi/60;
    params.deliverable1.speed.maxTsShaft_mPerSec    = 175;
    params.deliverable1.speed.maxTsPM_mPerSec       = 175;
    params.deliverable1.speed.maxTsFly_mPerSec      = 900;

    % Clearances
    params.deliverable1.clearance.axial_m          = 20e-3;
    params.deliverable1.clearance.radialFlywheel_m = 20e-3;
    params.deliverable1.clearance.radialOther_m    = 1e-3;

    % Material properties
    params.deliverable1.material.densFly_kgPerM3   = 1600;
    params.deliverable1.material.densSteel_kgPerM3 = 7850;
    params.deliverable1.material.densPM_kgPerM3    = 7850;

    % Balance grade
    params.deliverable1.balance.grade_ISO1940      = 'G2.5';

    % Controller gains
    params.control.current.k_p = 345;
    params.control.current.k_i = 2149;

    params.control.positionX.k_p    = 1.2639e8;
    params.control.positionX.k_i    = 1.16868e9;
    params.control.positionX.k_d    = 252790;
    params.control.positionX.omega_p_radPerSec = 3770;

    params.control.tilt.k_p    = 7.6992e7;
    params.control.tilt.k_i    = 1.18953e9;
    params.control.tilt.k_d    = 80294;
    params.control.tilt.omega_p_radPerSec = 6283;
end

function format = createPlotFormatStruct()
    format = struct();
    format.fontName  = 'Times New Roman';
    format.fontSize  = 12;
    format.figWidth  = 900;
    format.figHeight = 600;
    format.lineWidth = 2;
    format.color.primary   = [0.0000 0.4470 0.7410];
    format.color.secondary = [0.8500 0.3250 0.0980];
    format.color.ternary   = [0.4660 0.6740 0.1880];
end

function runDeliverable1_Peyton(params, format)
    p  = params.deliverable1;
    pc = params.control;

    soc_vec = linspace(0, 100, 100);

    r_fly   = p.geometry.flywheelDiameter_m / 2;
    h_fly   = p.geometry.flywheelAxialLength_m;
    d_shaft = p.geometry.shaftAndPMDiameter_m;
    r_shaft = d_shaft / 2;
    h_motor = p.geometry.motorAxialLength_m;
    t_mag   = p.geometry.magnetThickness_m;

    axial_clear = p.clearance.axial_m;

    ambParams = ambParameters(d_shaft, p.amb.ratedForce_N);
    h_shaft = (5 * axial_clear) + h_fly + h_motor + 2 * ambParams.axialLength;

    rho_fly   = p.material.densFly_kgPerM3;
    rho_steel = p.material.densSteel_kgPerM3;

    vol_shaft = pi * r_shaft^2 * h_shaft;
    vol_fly   = pi * (r_fly^2 - r_shaft^2) * h_fly;

    m_shaft = rho_steel * vol_shaft;
    m_fly   = rho_fly * vol_fly;
    m_rotating = m_shaft + m_fly;

    J = 0.5 * m_fly * (r_fly^2 - r_shaft^2) + 0.5 * m_shaft * r_shaft^2;

    omega_max_shaft = p.speed.maxTsShaft_mPerSec / r_shaft;
    omega_max_fly   = p.speed.maxTsFly_mPerSec   / r_fly;

    if omega_max_shaft <= omega_max_fly
        omega_max = omega_max_shaft;
    else
        omega_max = omega_max_fly;
    end

    omega_min = 0.5 * omega_max;

    KE_min = 0.5 * J * omega_min^2;
    KE_max = 0.5 * J * omega_max^2;

    [T_rated, P_rated_W] = computeRatedTorqueAndPower_Peyton(p, r_shaft, h_motor, t_mag, omega_min);

    [losses, T_rot_C, omega_vec] = computeLossesAndTemperatureVsSoc_Peyton( ...
        soc_vec, J, KE_min, KE_max, P_rated_W, T_rated, ...
        r_fly, h_fly, d_shaft, t_mag, p);

    % Plot and save
    plotLossesAndTemperatureVsSoc_Peyton(soc_vec, losses.rotor_W, losses.stator_W, ...
        losses.total_W, T_rot_C, format);

    fprintf('\n=== PEYTON VERSION RESULTS ===\n');
    fprintf('Rotating mass: %.2f kg\n', m_rotating);
    fprintf('Moment of inertia: %.4f kg*m^2\n', J);
    fprintf('Rated torque: %.2f Nm\n', T_rated);
    fprintf('Rated power: %.2f kW\n', P_rated_W/1000);
    fprintf('Max rotor losses: %.2f kW\n', max(losses.rotor_W)/1000);
    fprintf('Max stator losses: %.2f kW\n', max(losses.stator_W)/1000);
    fprintf('Max total losses: %.2f kW\n', max(losses.total_W)/1000);
    fprintf('Max temperature: %.1f C\n', max(T_rot_C));
    fprintf('Min temperature: %.1f C\n', min(T_rot_C));
end

function [T_rated, P_rated_W] = computeRatedTorqueAndPower_Peyton(p, r_shaft, h_motor, t_mag, omega_min)
    tau_rated = magneticShear(t_mag, 1);
    r_rotor = r_shaft + t_mag;  % Rotor radius includes magnets
    T_rated   = 2 * pi * r_rotor^2 * h_motor * tau_rated;
    P_rated_W = T_rated * omega_min;
end

function [losses, T_rot_C, omega_vec] = computeLossesAndTemperatureVsSoc_Peyton( ...
    soc_vec, J, KE_min, KE_max, P_rated_W, T_rated, ...
    r_fly, h_fly, d_shaft, t_mag, p)

    n = numel(soc_vec);
    losses.rotor_W  = zeros(1, n);
    losses.stator_W = zeros(1, n);
    losses.total_W  = zeros(1, n);
    T_rot_C   = zeros(1, n);
    omega_vec = zeros(1, n);

    D_rotor = d_shaft + 2 * t_mag;
    L_motor = p.geometry.motorAxialLength_m;

    epsilon = p.thermal.rotorEmissivity;
    sigmaSB = 5.67e-8;
    T_housing_K = p.thermal.housingTemp_C + 273.15;

    omega_min_from_P = P_rated_W / T_rated;

    for k = 1:n
        soc = soc_vec(k) / 100;
        KE    = KE_min + soc * (KE_max - KE_min);
        omega = sqrt(2 * KE / J);

        if abs(soc) < 1e-10 || soc_vec(k) == 0
            omega = omega_min_from_P;
        end

        omega_vec(k) = omega;

        Ipu = omega_min_from_P / omega;
        rotorSpeed_rpm = omega * 60 / (2*pi);

        losses.rotor_W(k)  = rotorLosses(t_mag, D_rotor, L_motor, Ipu, rotorSpeed_rpm);
        losses.stator_W(k) = statorLosses(t_mag, D_rotor, L_motor, Ipu, rotorSpeed_rpm);
        losses.total_W(k)  = losses.rotor_W(k) + losses.stator_W(k);

        r_o = r_fly;
        r_i = d_shaft / 2;
        A_surface = pi * (2 * r_o * h_fly) + 2 * pi * (r_o^2 - r_i^2);

        Q_gen = losses.rotor_W(k);
        T_K   = ((Q_gen / (epsilon * sigmaSB * A_surface)) + T_housing_K^4)^(1/4);
        T_rot_C(k) = T_K - 273.15;
    end
end

function plotLossesAndTemperatureVsSoc_Peyton(soc_vec, rotorLoss_W, statorLoss_W, ...
        totalLoss_W, T_rot_C, format)

    figure('Position', [100, 100, 1000, 450]);

    subplot(1,2,1);
    plot(soc_vec, rotorLoss_W/1e3, 'r-', 'LineWidth', format.lineWidth); hold on;
    plot(soc_vec, statorLoss_W/1e3, 'b-', 'LineWidth', format.lineWidth);
    plot(soc_vec, totalLoss_W/1e3, 'k-', 'LineWidth', format.lineWidth);
    grid on;
    xlabel('State of Charge [%]', 'FontSize', 12);
    ylabel('Power Loss [kW]', 'FontSize', 12);
    title('PEYTON: Motor Losses at Rated Power vs SoC', 'FontSize', 13);
    legend('Rotor Losses', 'Stator Losses', 'Total Losses', 'Location', 'best');

    subplot(1,2,2);
    plot(soc_vec, T_rot_C, 'k-', 'LineWidth', format.lineWidth); hold on;
    yline(100, 'r--', 'LineWidth', 2, 'Label', 'Max Safe (100 C)');
    grid on;
    xlabel('State of Charge [%]', 'FontSize', 12);
    ylabel('Rotor Temperature [C]', 'FontSize', 12);
    title('PEYTON: Rotor Temperature vs SoC', 'FontSize', 13);

    saveas(gcf, 'peyton_part1a_losses_temperature.png');
    fprintf('Saved: peyton_part1a_losses_temperature.png\n');
end
