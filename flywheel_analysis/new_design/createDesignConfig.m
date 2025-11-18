function design = createDesignConfig(omega_max, magnet_thickness, energy_required, P_rated)
% createDesignConfig - Create configuration for a new flywheel design
%
% Syntax: design = createDesignConfig(omega_max, magnet_thickness,
%                                     energy_required, P_rated)
%
% Inputs:
%    omega_max        - Maximum angular velocity [rad/s]
%    magnet_thickness - PM thickness [m]
%    energy_required  - Required usable energy [J]
%    P_rated          - Rated power [W]
%
% Outputs:
%    design - Configuration structure (similar to baseline_config)

design = struct();

% Operating parameters
design.omega_max = omega_max;
design.P_rated = P_rated;
design.energy_required = abs(energy_required) * 2; % Size for margin
design.magnet_thickness = magnet_thickness;

% Calculate dimensions
dims = calculateFlywheelDimensions(omega_max, magnet_thickness, design.energy_required);
design.dims = dims;

% Calculate mass and inertia
[mass, I, mass_breakdown] = calculateRotatingGroupMass(dims);
design.mass_total = mass;
design.I_total = I;
design.mass_breakdown = mass_breakdown;

% Operating conditions
design.T_ambient = 25; % °C

% AMB parameters
F_amb_rating = 2 * mass * 9.81; % N (2x rotating group weight)
design.F_amb_rating = F_amb_rating;

% Get AMB parameters
amb_params_top = ambParameters(dims.r_shaft_outer, F_amb_rating);
amb_params_bottom = ambParameters(dims.r_shaft_outer, F_amb_rating);

design.amb_top = amb_params_top;
design.amb_bottom = amb_params_bottom;

% Controller (will be designed separately)
design.controller = struct();
design.controller.designed = false;

end
