function controller = design_amb_controller(amb_params, m_rotor, target_bandwidth, damping_ratio)
% design_amb_controller - Design cascaded position and current controllers for AMB
%
% Syntax: controller = design_amb_controller(amb_params, m_rotor,
%                                            target_bandwidth, damping_ratio)
%
% Inputs:
%    amb_params       - AMB parameters structure from ambParameters()
%    m_rotor          - Rotor mass supported by THIS bearing [kg]
%    target_bandwidth - Desired closed-loop bandwidth [Hz]
%    damping_ratio    - Desired damping ratio (typically 0.7-1.0)
%
% Outputs:
%    controller - Structure containing:
%       .Gc_pos   - Position controller transfer function
%       .Gc_curr  - Current controller transfer function
%       .K_p      - Position proportional gain
%       .K_d      - Position derivative gain
%       .K_i_curr - Current integral gain
%       .K_p_curr - Current proportional gain
%       .sys_cl   - Closed-loop system transfer function
%       .bandwidth - Achieved bandwidth [Hz]
%
% Notes:
%    AMB system model:
%    - Mechanical equation: m*x'' = F_amb + F_dist
%    - Magnetic force: F_amb = k_s * x + k_i * i
%    - Electromagnet dynamics: L*di/dt + R*i = v
%    - Control architecture: Cascaded position and current control

fprintf('Designing AMB Controller...\n');

% Extract AMB parameters
k_s = amb_params.k_s;       % Negative stiffness [N/m]
k_i = amb_params.k_i;       % Force constant [N/A]
L = amb_params.L;           % Inductance [H]
R = amb_params.R;           % Resistance [Ohm]
i_max = amb_params.i_max;   % Max control current [A]

fprintf('  AMB Parameters:\n');
fprintf('    k_s = %.0f N/m (negative stiffness)\n', k_s);
fprintf('    k_i = %.1f N/A (force constant)\n', k_i);
fprintf('    L = %.4f H (inductance)\n', L);
fprintf('    R = %.2f Ohm (resistance)\n', R);
fprintf('    i_max = %.1f A (max control current)\n', i_max);

%% Current Controller Design (Inner Loop)
% Plant: G_i(s) = i(s)/v(s) = 1/(L*s + R)
% PI controller: Gc_i(s) = K_p_curr + K_i_curr/s

% Design for current loop bandwidth >> position loop bandwidth
omega_curr_bandwidth = 2*pi * target_bandwidth * 10; % rad/s (10x faster)

% PI controller zero cancels plant pole
tau_elec = L / R;
K_i_curr = L * omega_curr_bandwidth;
K_p_curr = K_i_curr * tau_elec;

fprintf('  Current Controller:\n');
fprintf('    K_p_curr = %.2f\n', K_p_curr);
fprintf('    K_i_curr = %.2f\n', K_i_curr);
fprintf('    Current loop bandwidth: %.1f Hz\n', omega_curr_bandwidth/(2*pi));

% Current controller transfer function
s = tf('s');
Gc_curr = K_p_curr + K_i_curr/s;

%% Position Controller Design (Outer Loop)
% With fast current loop, approximate: i ≈ i_cmd
% Mechanical plant with negative stiffness:
% G_pos(s) = x(s)/F(s) = 1/(m*s^2 - k_s)
%
% With force F = k_i * i_cmd, we have:
% x(s)/i_cmd(s) = k_i/(m*s^2 - k_s)
%
% PD controller: Gc_pos(s) = K_p + K_d*s
% This generates current command: i_cmd = -Gc_pos(s) * x

% Desired closed-loop characteristic equation:
% s^2 + 2*zeta*omega_n*s + omega_n^2 = 0
omega_n = 2*pi * target_bandwidth; % Natural frequency [rad/s]
zeta = damping_ratio;

% Closed-loop equation with PD controller:
% m*s^2*x = k_s*x + k_i*i_cmd
% m*s^2*x = k_s*x - k_i*(K_p + K_d*s)*x
% m*s^2*x + k_i*K_d*s*x + (k_i*K_p - k_s)*x = 0
% s^2 + (k_i*K_d/m)*s + (k_i*K_p - k_s)/m = 0

% Matching coefficients:
% k_i*K_d/m = 2*zeta*omega_n
% (k_i*K_p - k_s)/m = omega_n^2

K_d = 2*zeta*omega_n*m/k_i;
K_p = (m*omega_n^2 + k_s)/k_i;

fprintf('  Position Controller:\n');
fprintf('    K_p = %.4f A/m\n', K_p);
fprintf('    K_d = %.4f A*s/m\n', K_d);
fprintf('    Target bandwidth: %.1f Hz\n', target_bandwidth);
fprintf('    Damping ratio: %.2f\n', damping_ratio);

% Position controller transfer function
Gc_pos = K_p + K_d*s;

%% Closed-Loop System
% For analysis, create closed-loop transfer function
% x(s)/F_dist(s) where F_dist is disturbance force

% Open-loop plant (position to force)
G_plant = k_i/(m*s^2 - k_s);

% Closed-loop transfer function (disturbance to position)
sys_cl = 1/(m*s^2 + k_i*K_d*s + (k_i*K_p - k_s));

% Verify bandwidth
[mag, phase, wout] = bode(sys_cl, logspace(-1, 4, 1000));
mag_db = 20*log10(squeeze(mag));
idx_3db = find(mag_db <= -3, 1, 'first');
if ~isempty(idx_3db)
    bandwidth_achieved = wout(idx_3db) / (2*pi);
else
    bandwidth_achieved = NaN;
end

fprintf('    Achieved bandwidth: %.1f Hz\n', bandwidth_achieved);

%% Package results
controller = struct();
controller.Gc_pos = Gc_pos;
controller.Gc_curr = Gc_curr;
controller.K_p = K_p;
controller.K_d = K_d;
controller.K_i_curr = K_i_curr;
controller.K_p_curr = K_p_curr;
controller.sys_cl = sys_cl;
controller.bandwidth = bandwidth_achieved;
controller.omega_n = omega_n;
controller.zeta = zeta;

% AMB system for reference
controller.G_plant = G_plant;
controller.amb_params = amb_params;
controller.m_rotor = m_rotor;

fprintf('Controller design complete.\n\n');

end
