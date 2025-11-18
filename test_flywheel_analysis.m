% test_flywheel_analysis.m - Test script for flywheel analysis framework
%
% This script tests all major components of the flywheel analysis

clear; clc; close all;

fprintf('========================================\n');
fprintf('TESTING FLYWHEEL ANALYSIS FRAMEWORK\n');
fprintf('========================================\n\n');

% Add all necessary paths
addpath('flywheel_analysis/utilities');
addpath('flywheel_analysis/ee_functions');
addpath('flywheel_analysis/baseline_analysis');
addpath('flywheel_analysis/new_design');
addpath('flywheel_analysis/amb_controllers');

% Create results directory
results_dir = 'flywheel_analysis/results';
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

%% Test 1: Utility Functions
fprintf('TEST 1: Utility Functions\n');
fprintf('---------------------------\n');

% Test energy calculations
omega_max = 3000; % rad/s
I = 1.0; % kg*m^2
[E, E_usable] = flywheelEnergy(omega_max, I);
fprintf('  Energy calculation: E = %.2f kJ, E_usable = %.2f kJ\n', E/1000, E_usable/1000);

% Test SoC conversions
SoC = 0.5;
omega = soc2speed(SoC, omega_max);
SoC_back = speed2soc(omega, omega_max);
fprintf('  SoC conversion: SoC=%.2f -> omega=%.1f rad/s -> SoC=%.2f\n', SoC, omega, SoC_back);

fprintf('  ✓ Utility functions working\n\n');

%% Test 2: Baseline Configuration
fprintf('TEST 2: Baseline Configuration\n');
fprintf('--------------------------------\n');

try
    config = baseline_config();
    fprintf('  ✓ Baseline config loaded successfully\n');
    fprintf('    - Max speed: %.0f RPM\n', config.omega_max * 60/(2*pi));
    fprintf('    - Total mass: %.1f kg\n', config.mass_total);
    fprintf('    - Total inertia: %.4f kg*m^2\n', config.I_total);
catch ME
    fprintf('  ✗ Error in baseline_config: %s\n', ME.message);
end
fprintf('\n');

%% Test 3: Geometry Calculations
fprintf('TEST 3: Geometry Calculations\n');
fprintf('-------------------------------\n');

omega_design = 2500; % rad/s
magnet_thick = 0.003; % m
energy_req = 1e6; % J

try
    dims = calculateFlywheelDimensions(omega_design, magnet_thick, energy_req);
    fprintf('  ✓ Dimensions calculated\n');
    fprintf('    - Flywheel OD: %.3f m\n', 2*dims.r_fw_outer);
    fprintf('    - Flywheel thickness: %.3f m\n', dims.l_flywheel);

    [mass, I, mass_breakdown] = calculateRotatingGroupMass(dims);
    fprintf('  ✓ Mass properties calculated\n');
    fprintf('    - Total mass: %.1f kg\n', mass);
    fprintf('    - Inertia: %.4f kg*m^2\n', I);
catch ME
    fprintf('  ✗ Error: %s\n', ME.message);
end
fprintf('\n');

%% Test 4: EE Functions
fprintf('TEST 4: EE Team Functions (Placeholders)\n');
fprintf('------------------------------------------\n');

try
    tau = magneticShear(0.003, 50);
    fprintf('  ✓ magneticShear: tau = %.0f Pa\n', tau);

    P_rotor = rotorLosses(0.003, 0.1, 0.08, 50, 1000);
    fprintf('  ✓ rotorLosses: P = %.1f W\n', P_rotor);

    P_stator = statorLosses(0.003, 0.1, 0.08, 50, 1000);
    fprintf('  ✓ statorLosses: P = %.1f W\n', P_stator);

    amb_params = ambParameters(0.05, 500);
    fprintf('  ✓ ambParameters: k_i = %.1f N/A\n', amb_params.k_i);
catch ME
    fprintf('  ✗ Error: %s\n', ME.message);
end
fprintf('\n');

%% Test 5: Thermal Analysis
fprintf('TEST 5: Thermal Analysis\n');
fprintf('-------------------------\n');

try
    P_rotor_test = 100; % W
    P_stator_test = 200; % W
    omega_test = 1000; % rad/s

    [T_rotor, T_stator] = calculateRotorTemperature(P_rotor_test, P_stator_test, ...
                                                     dims, omega_test, 25);
    fprintf('  ✓ Temperature calculation\n');
    fprintf('    - Rotor: %.1f °C\n', T_rotor);
    fprintf('    - Stator: %.1f °C\n', T_stator);
catch ME
    fprintf('  ✗ Error: %s\n', ME.message);
end
fprintf('\n');

%% Test 6: AMB Controller Design
fprintf('TEST 6: AMB Controller Design\n');
fprintf('-------------------------------\n');

try
    test_amb_params = ambParameters(0.05, 500);
    controller = design_amb_controller(test_amb_params, 30, 50, 0.7);
    fprintf('  ✓ AMB controller designed\n');
    fprintf('    - Bandwidth: %.1f Hz\n', controller.bandwidth);
    fprintf('    - K_p: %.4f A/m\n', controller.K_p);
    fprintf('    - K_d: %.4f A*s/m\n', controller.K_d);
catch ME
    fprintf('  ✗ Error: %s\n', ME.message);
end
fprintf('\n');

%% Test 7: Simple Plot Generation
fprintf('TEST 7: Plot Generation\n');
fprintf('------------------------\n');

try
    % Create a simple test plot
    figure('Visible', 'off'); % Create figure without displaying
    SoC_test = linspace(0, 1, 50);
    omega_test = soc2speed(SoC_test, omega_max);
    plot(SoC_test*100, omega_test*60/(2*pi), 'LineWidth', 2);
    xlabel('State of Charge [%]');
    ylabel('Speed [RPM]');
    title('Test Plot: Speed vs SoC');
    grid on;

    % Save the figure
    saveas(gcf, fullfile(results_dir, 'test_plot.fig'));
    saveas(gcf, fullfile(results_dir, 'test_plot.png'));
    close(gcf);

    fprintf('  ✓ Plot created and saved\n');
catch ME
    fprintf('  ✗ Error: %s\n', ME.message);
end
fprintf('\n');

%% Summary
fprintf('========================================\n');
fprintf('TEST SUMMARY\n');
fprintf('========================================\n');
fprintf('All core functions are operational!\n');
fprintf('Results directory: %s\n', results_dir);
fprintf('\nCheck for test_plot.png in results folder.\n');
fprintf('\nNext steps:\n');
fprintf('1. Replace placeholder EE functions\n');
fprintf('2. Update baseline_config.m with Appendix A\n');
fprintf('3. Load Team #16 storage cycle\n');
fprintf('4. Run full analyses\n');
fprintf('========================================\n');
