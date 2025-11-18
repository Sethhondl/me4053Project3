function config = baseline_config()
% baseline_config - Configuration parameters for baseline flywheel system
%
% PLACEHOLDER - Replace with actual Appendix A specifications
%
% Syntax: config = baseline_config()
%
% Outputs:
%    config - Structure containing baseline system parameters

%% BASELINE SYSTEM PARAMETERS (ASSUMED - REPLACE WITH APPENDIX A)
% These are reasonable assumptions based on typical flywheel systems

config = struct();

% Operating parameters
config.omega_max = 30000 * (2*pi/60);  % Max speed [rad/s] (30,000 RPM assumed)
config.P_rated = 100e3;                 % Rated power [W] (100 kW assumed)
config.energy_required = 5e6;           % Required usable energy [J] (1.39 kWh assumed)

% Component dimensions (will be calculated, but can specify if known)
config.magnet_thickness = 0.004;        % PM thickness [m] (4 mm assumed)

% Calculated dimensions (based on constraints)
dims = calculateFlywheelDimensions(config.omega_max, config.magnet_thickness, ...
                                   config.energy_required);
config.dims = dims;

% Calculate mass and inertia
[mass, I, mass_breakdown] = calculateRotatingGroupMass(dims);
config.mass_total = mass;
config.I_total = I;
config.mass_breakdown = mass_breakdown;

% Operating conditions
config.T_ambient = 25;                  % Ambient temperature [°C]

% AMB parameters
% Force rating = 2 * rotating group weight
F_amb_rating = 2 * mass * 9.81;         % N
config.F_amb_rating = F_amb_rating;

% Get AMB parameters for top and bottom bearings
amb_params_top = ambParameters(dims.r_shaft_outer, F_amb_rating);
amb_params_bottom = ambParameters(dims.r_shaft_outer, F_amb_rating);

config.amb_top = amb_params_top;
config.amb_bottom = amb_params_bottom;

% Controller parameters (placeholder - will be designed)
config.controller = struct();
config.controller.designed = false;

%% Display warning
fprintf('\n');
fprintf('========================================================\n');
fprintf('WARNING: Using ASSUMED baseline system parameters\n');
fprintf('Replace with actual Appendix A specifications!\n');
fprintf('========================================================\n');
fprintf('Assumed parameters:\n');
fprintf('  Max speed: %.0f RPM\n', config.omega_max * 60/(2*pi));
fprintf('  Rated power: %.0f kW\n', config.P_rated / 1000);
fprintf('  Usable energy: %.2f kWh\n', config.energy_required / 3.6e6);
fprintf('  Magnet thickness: %.1f mm\n', config.magnet_thickness * 1000);
fprintf('  Total mass: %.1f kg\n', config.mass_total);
fprintf('  Total inertia: %.4f kg*m^2\n', config.I_total);
fprintf('========================================================\n');
fprintf('\n');

end
