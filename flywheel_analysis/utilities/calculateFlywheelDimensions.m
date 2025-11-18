function dims = calculateFlywheelDimensions(omega_max, magnet_thickness, energy_required)
% calculateFlywheelDimensions - Size flywheel components based on constraints
%
% Syntax: dims = calculateFlywheelDimensions(omega_max, magnet_thickness, energy_required)
%
% Inputs:
%    omega_max       - Maximum angular velocity [rad/s]
%    magnet_thickness - PM thickness [m]
%    energy_required  - Required usable energy storage [J]
%
% Outputs:
%    dims - Structure containing component dimensions:
%       .r_shaft_inner  - Shaft inner radius [m]
%       .r_shaft_outer  - Shaft outer radius [m]
%       .r_pm_inner     - PM inner radius [m]
%       .r_pm_outer     - PM outer radius [m]
%       .r_fw_inner     - Flywheel inner radius [m]
%       .r_fw_outer     - Flywheel outer radius [m]
%       .l_shaft        - Shaft length [m]
%       .l_motor        - Motor/generator axial length [m]
%       .l_amb          - AMB axial length (each) [m]
%       .l_flywheel     - Flywheel thickness [m]
%       .total_length   - Total rotating assembly length [m]
%
% Notes:
%    Constraints from Table 1:
%    - Max steel shaft tip speed: 175 m/s
%    - Max PM tip speed: 175 m/s
%    - Max composite flywheel tip speed: 900 m/s
%    - Axial clearances between components: 20 mm

% Material properties
rho_steel = 7850;     % kg/m^3
rho_composite = 1600; % kg/m^3
rho_pm = 7850;        % kg/m^3

% Initialize structure
dims = struct();

% Determine radii based on tip speed constraints
v_max_shaft = 175;      % m/s
v_max_pm = 175;         % m/s
v_max_flywheel = 900;   % m/s

dims.r_shaft_outer = v_max_shaft / omega_max;  % m
dims.r_pm_inner = dims.r_shaft_outer;
dims.r_pm_outer = min(v_max_pm / omega_max, dims.r_pm_inner + magnet_thickness);

% Motor/generator active length (estimate based on power requirements)
% Will be refined in actual design
dims.l_motor = 0.08; % m (80 mm initial estimate)

% Flywheel sizing - maximize energy storage
dims.r_fw_inner = dims.r_pm_outer;
dims.r_fw_outer = v_max_flywheel / omega_max;

% Flywheel thickness - size for required energy
% For a thick disk: I = 0.5 * m * (r_i^2 + r_o^2)
% where m = rho * pi * (r_o^2 - r_i^2) * h
% Energy: E = 0.375 * I * omega_max^2 (usable energy)

% Estimate required thickness
% E = 0.375 * 0.5 * m * (r_i^2 + r_o^2) * omega_max^2
% E = 0.1875 * rho * pi * (r_o^2 - r_i^2) * h * (r_i^2 + r_o^2) * omega_max^2

r_i = dims.r_fw_inner;
r_o = dims.r_fw_outer;
if energy_required > 0
    dims.l_flywheel = energy_required / ...
        (0.1875 * rho_composite * pi * (r_o^2 - r_i^2) * (r_i^2 + r_o^2) * omega_max^2);
else
    dims.l_flywheel = 0.1; % m (100 mm default)
end

% Ensure reasonable minimum thickness
dims.l_flywheel = max(dims.l_flywheel, 0.05); % minimum 50 mm

% Shaft dimensions
dims.r_shaft_inner = 0.01; % 10 mm inner radius (hollow shaft)

% Shaft length - extends through motor and bearings
dims.l_amb = 0.03; % 30 mm per bearing (will be refined)
clearance = 0.020; % 20 mm between components

dims.l_shaft = dims.l_flywheel + 2*clearance + dims.l_motor + ...
               2*clearance + 2*dims.l_amb + 2*clearance;

% Total rotating assembly length
dims.total_length = dims.l_shaft;

end
