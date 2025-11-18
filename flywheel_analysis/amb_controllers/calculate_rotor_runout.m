function runout = calculate_rotor_runout(config, controller_top, controller_bottom, omega_array, imbalance)
% calculate_rotor_runout - Calculate rotor runout due to mass imbalance
%
% Syntax: runout = calculate_rotor_runout(config, controller_top,
%                                         controller_bottom, omega_array, imbalance)
%
% Inputs:
%    config           - Configuration structure
%    controller_top    - Top AMB controller structure
%    controller_bottom - Bottom AMB controller structure
%    omega_array       - Rotor speed array [rad/s]
%    imbalance         - Mass imbalance [kg*m] (mass × eccentricity)
%
% Outputs:
%    runout - Structure containing:
%       .omega         - Speed vector [rad/s]
%       .rpm           - Speed vector [RPM]
%       .x_max         - Maximum radial displacement [m]
%       .x_max_mm      - Maximum radial displacement [mm]
%       .F_imbalance   - Imbalance force magnitude [N]
%
% Notes:
%    Imbalance force: F = m*e*ω² (centrifugal force)
%    where m*e is the imbalance (mass × eccentricity)
%
%    The rotor response is the forced vibration at the rotation frequency

fprintf('Calculating rotor runout due to mass imbalance...\n');

% Extract AMB parameters
k_s = controller_top.amb_params.k_s;
k_i = controller_top.amb_params.k_i;
K_p = controller_top.K_p;
K_d = controller_top.K_d;

m_total = config.mass_total;

% Closed-loop transfer function for radial displacement
% From force to displacement: x(s)/F(s)
s = tf('s');
H_radial = controller_top.sys_cl;

% For each speed, calculate imbalance force and resulting displacement
n_speeds = length(omega_array);
x_max = zeros(n_speeds, 1);
F_imbalance = zeros(n_speeds, 1);

for i = 1:n_speeds
    omega = omega_array(i);

    % Imbalance force amplitude (rotating at ω)
    F_imb = imbalance * omega^2; % N
    F_imbalance(i) = F_imb;

    % Evaluate transfer function at s = jω
    s_eval = 1j * omega;

    % Using the closed-loop transfer function
    % H_radial = 1/(m*s^2 + k_i*K_d*s + (k_i*K_p - k_s))
    m_eff = m_total; % Effective mass for radial mode

    H_val = 1/(m_eff*s_eval^2 + k_i*K_d*s_eval + (k_i*K_p - k_s));

    % Displacement amplitude
    x_amplitude = abs(H_val) * F_imb; % m

    x_max(i) = x_amplitude;
end

% Convert to mm and RPM for convenience
x_max_mm = x_max * 1000; % mm
rpm_array = omega_array * 60/(2*pi); % RPM

%% Package results
runout = struct();
runout.omega = omega_array;
runout.rpm = rpm_array;
runout.x_max = x_max;
runout.x_max_mm = x_max_mm;
runout.F_imbalance = F_imbalance;

fprintf('Rotor runout calculation complete.\n\n');

end
