function [mass_total, I_total, mass_breakdown] = calculateRotatingGroupMass(dims)
% calculateRotatingGroupMass - Calculate mass and inertia of rotating assembly
%
% Syntax: [mass_total, I_total, mass_breakdown] = calculateRotatingGroupMass(dims)
%
% Inputs:
%    dims - Structure from calculateFlywheelDimensions
%
% Outputs:
%    mass_total - Total rotating group mass [kg]
%    I_total    - Total mass moment of inertia about rotation axis [kg*m^2]
%    mass_breakdown - Structure with individual component masses:
%       .shaft     - Steel shaft mass [kg]
%       .magnets   - Permanent magnet mass [kg]
%       .flywheel  - Composite flywheel mass [kg]
%       .rotor_back_iron - Motor rotor back iron mass [kg]
%       .amb_rotors     - AMB rotor masses (both bearings) [kg]
%
% Notes:
%    Inertia formulas:
%    - Hollow cylinder: I = 0.5 * m * (r_i^2 + r_o^2)
%    - Thick disk: I = 0.5 * m * (r_i^2 + r_o^2)

% Material densities
rho_steel = 7850;     % kg/m^3
rho_composite = 1600; % kg/m^3
rho_pm = 7850;        % kg/m^3

% Initialize breakdown structure
mass_breakdown = struct();

%% Shaft mass and inertia
r_i = dims.r_shaft_inner;
r_o = dims.r_shaft_outer;
l = dims.l_shaft;

V_shaft = pi * (r_o^2 - r_i^2) * l;
mass_breakdown.shaft = rho_steel * V_shaft;
I_shaft = 0.5 * mass_breakdown.shaft * (r_i^2 + r_o^2);

%% Permanent magnets mass and inertia
r_i = dims.r_pm_inner;
r_o = dims.r_pm_outer;
l = dims.l_motor;

V_pm = pi * (r_o^2 - r_i^2) * l;
mass_breakdown.magnets = rho_pm * V_pm;
I_pm = 0.5 * mass_breakdown.magnets * (r_i^2 + r_o^2);

%% Motor rotor back iron (steel between shaft and magnets)
% Simplified: assume thin steel backing for magnets
t_back_iron = 0.003; % 3 mm backing
r_i = dims.r_shaft_outer;
r_o = r_i + t_back_iron;
l = dims.l_motor;

V_back_iron = pi * (r_o^2 - r_i^2) * l;
mass_breakdown.rotor_back_iron = rho_steel * V_back_iron;
I_back_iron = 0.5 * mass_breakdown.rotor_back_iron * (r_i^2 + r_o^2);

%% Flywheel mass and inertia
r_i = dims.r_fw_inner;
r_o = dims.r_fw_outer;
l = dims.l_flywheel;

V_flywheel = pi * (r_o^2 - r_i^2) * l;
mass_breakdown.flywheel = rho_composite * V_flywheel;
I_flywheel = 0.5 * mass_breakdown.flywheel * (r_i^2 + r_o^2);

%% AMB rotors (laminated steel targets)
% Simplified: assume 2 bearings, each with steel rotor target
t_amb_rotor = 0.005; % 5 mm thickness estimate
r_i = dims.r_shaft_outer * 0.8; % slightly smaller than shaft OD
r_o = dims.r_shaft_outer;
l_total = 2 * dims.l_amb; % two bearings

V_amb = pi * (r_o^2 - r_i^2) * l_total;
mass_breakdown.amb_rotors = rho_steel * V_amb;
I_amb = 0.5 * mass_breakdown.amb_rotors * (r_i^2 + r_o^2);

%% Total mass and inertia
mass_total = mass_breakdown.shaft + mass_breakdown.magnets + ...
             mass_breakdown.flywheel + mass_breakdown.rotor_back_iron + ...
             mass_breakdown.amb_rotors;

I_total = I_shaft + I_pm + I_flywheel + I_back_iron + I_amb;

end
