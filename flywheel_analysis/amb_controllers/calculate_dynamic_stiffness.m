function stiffness = calculate_dynamic_stiffness(config, controller_top, controller_bottom, freq_array)
% calculate_dynamic_stiffness - Calculate AMB system dynamic stiffness vs frequency
%
% Syntax: stiffness = calculate_dynamic_stiffness(config, controller_top,
%                                                 controller_bottom, freq_array)
%
% Inputs:
%    config           - Configuration structure
%    controller_top   - Top AMB controller structure
%    controller_bottom - Bottom AMB controller structure
%    freq_array       - Frequency vector [Hz]
%
% Outputs:
%    stiffness - Structure containing:
%       .freq        - Frequency vector [Hz]
%       .K_radial    - Radial stiffness magnitude [N/m]
%       .K_tilting   - Tilting stiffness magnitude [N*m/rad]
%       .phase_radial  - Radial stiffness phase [deg]
%       .phase_tilting - Tilting stiffness phase [deg]
%
% Notes:
%    Dynamic stiffness K(jω) = -F(jω)/x(jω)
%    For two-bearing system:
%    - Radial mode: both bearings move in same direction
%    - Tilting mode: bearings move in opposite directions

fprintf('Calculating dynamic stiffness...\n');

% Extract bearing locations (simplified: symmetric about flywheel center)
dims = config.dims;
L_bearing_spacing = dims.l_flywheel + 2*0.020 + dims.l_motor; % m

% Mass properties
m_total = config.mass_total;
m_per_bearing = m_total / 2; % Assume equal load distribution

% Moments of inertia (for tilting mode)
% Simplified: assume flywheel dominates (thick disk)
r_fw_outer = dims.r_fw_outer;
r_fw_inner = dims.r_fw_inner;
l_fw = dims.l_flywheel;
rho_comp = 1600; % kg/m^3
m_fw = rho_comp * pi * (r_fw_outer^2 - r_fw_inner^2) * l_fw;

% Transverse moment of inertia (for tilting/wobble mode)
I_transverse = m_fw * (3*(r_fw_inner^2 + r_fw_outer^2)/12 + l_fw^2/12);

%% Radial (Translational) Mode
% Both bearings act in parallel for translation
% Effective stiffness = K_top + K_bottom

% Closed-loop transfer function from force to displacement
% For each bearing: x(s)/F(s) = 1/(m*s^2 + k_i*K_d*s + (k_i*K_p - k_s))

% Top bearing
m_top = m_per_bearing;
k_s_top = controller_top.amb_params.k_s;
k_i_top = controller_top.amb_params.k_i;
K_p_top = controller_top.K_p;
K_d_top = controller_top.K_d;

% Transfer function: x/F for top bearing
s = tf('s');
H_top = 1/(m_top*s^2 + k_i_top*K_d_top*s + (k_i_top*K_p_top - k_s_top));

% Bottom bearing (assume symmetric)
H_bottom = H_top; % Simplified

% Combined system for radial mode
H_radial = H_top * H_bottom / (H_top + H_bottom); % Parallel combination

% Dynamic stiffness: K(jω) = -F/x = -1/H(jω)
omega_array = 2*pi*freq_array; % rad/s
[mag_radial, phase_radial] = bode(1/H_radial, omega_array);
K_radial = squeeze(mag_radial); % N/m
phase_radial = squeeze(phase_radial); % degrees

%% Tilting (Rotational) Mode
% For tilting about center, bearings act with moment arm
% Moment arm = L_bearing_spacing/2

% For small angle θ:
% Bearing displacement: x_bearing = θ * (L_bearing_spacing/2)
% Bearing forces: F_top, F_bottom (opposite directions)
% Total moment: M = F_top * (L/2) + F_bottom * (L/2)

% Equation of motion for rotation:
% I_transverse * θ_ddot = M = (F_top + F_bottom) * (L/2)

% With bearing controllers acting on displacement x = θ*(L/2):
% F_bearing = -K_bearing(s) * x = -K_bearing(s) * θ * (L/2)

% Effective rotational stiffness:
% K_θ(s) = -M/θ = K_bearing(s) * (L/2)^2 * 2 bearings

L_arm = L_bearing_spacing / 2;

% Transfer function from moment to angle
% θ/M = 1/(I*s^2 + K_θ_eff)
% where K_θ_eff = k_i*K_p_top*(L_arm)^2 - k_s_top*(L_arm)^2 (for each bearing, x2)

K_theta_static = 2 * (k_i_top*K_p_top - k_s_top) * L_arm^2; % N*m/rad
D_theta = 2 * k_i_top*K_d_top * L_arm^2; % N*m*s/rad

H_tilting = 1/(I_transverse*s^2 + D_theta*s + K_theta_static);

% Dynamic stiffness for tilting
[mag_tilting, phase_tilting] = bode(1/H_tilting, omega_array);
K_tilting = squeeze(mag_tilting); % N*m/rad
phase_tilting = squeeze(phase_tilting); % degrees

%% Package results
stiffness = struct();
stiffness.freq = freq_array;
stiffness.K_radial = K_radial;
stiffness.K_tilting = K_tilting;
stiffness.phase_radial = phase_radial;
stiffness.phase_tilting = phase_tilting;
stiffness.H_radial = H_radial;
stiffness.H_tilting = H_tilting;

fprintf('Dynamic stiffness calculation complete.\n\n');

end
