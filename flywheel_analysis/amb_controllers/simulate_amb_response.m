function response = simulate_amb_response(controller, F_dist, omega_rotor, t_sim)
% simulate_amb_response - Simulate AMB system response to step disturbance
%
% Syntax: response = simulate_amb_response(controller, F_dist, omega_rotor, t_sim)
%
% Inputs:
%    controller  - Controller structure from design_amb_controller()
%    F_dist      - Step disturbance force magnitude [N]
%    omega_rotor - Rotor rotational speed [rad/s]
%    t_sim       - Simulation time vector [s]
%
% Outputs:
%    response - Structure containing:
%       .t     - Time vector [s]
%       .x     - Position response [m]
%       .v     - Velocity response [m/s]
%       .i_cmd - Current command [A]
%       .i     - Actual current [A]
%       .F_amb - AMB force [N]
%
% Notes:
%    Gyroscopic effects at high speeds are simplified in this model

% Extract parameters
m = controller.m_rotor;
k_s = controller.amb_params.k_s;
k_i = controller.amb_params.k_i;
L = controller.amb_params.L;
R = controller.amb_params.R;
K_p = controller.K_p;
K_d = controller.K_d;
K_p_curr = controller.K_p_curr;
K_i_curr = controller.K_i_curr;
i_max = controller.amb_params.i_max;

%% State-space model
% States: [x, x_dot, i, i_integral]
% x = position [m]
% x_dot = velocity [m/s]
% i = coil current [A]
% i_integral = integral of current error [A*s]

% Position controller generates current command
% i_cmd = -(K_p*x + K_d*x_dot)

% Current controller generates voltage
% v = K_p_curr*(i_cmd - i) + K_i_curr*i_integral

% Dynamics:
% m*x_ddot = k_s*x + k_i*i + F_dist
% L*i_dot = -R*i + v
% i_integral_dot = i_cmd - i

% State-space: dx/dt = A*x + B*u
% where u = F_dist

% State vector: [x; x_dot; i; i_int]
A = zeros(4, 4);
B = zeros(4, 1);

% x_dot equation
A(1, 2) = 1;

% x_ddot equation: x_ddot = (k_s*x + k_i*i + F_dist)/m
A(2, 1) = k_s/m;
A(2, 3) = k_i/m;
B(2) = 1/m;

% i_dot equation: i_dot = (-R*i + v)/L
% where v = K_p_curr*(i_cmd - i) + K_i_curr*i_int
%         = K_p_curr*(-K_p*x - K_d*x_dot - i) + K_i_curr*i_int
A(3, 1) = -K_p_curr*K_p/L;
A(3, 2) = -K_p_curr*K_d/L;
A(3, 3) = -(R + K_p_curr)/L;
A(3, 4) = K_i_curr/L;

% i_int_dot equation: i_int_dot = i_cmd - i = -K_p*x - K_d*x_dot - i
A(4, 1) = -K_p;
A(4, 2) = -K_d;
A(4, 3) = -1;

% Create state-space system
sys_ss = ss(A, B, eye(4), zeros(4,1));

% Initial conditions (at equilibrium)
x0 = [0; 0; 0; 0];

% Simulate step response
[y, t, x_states] = lsim(sys_ss, F_dist * ones(size(t_sim)), t_sim, x0);

% Extract states
x_pos = y(:, 1);        % Position [m]
x_vel = y(:, 2);        % Velocity [m/s]
i_actual = y(:, 3);     % Current [A]
i_int = y(:, 4);        % Current integral [A*s]

% Calculate current command
i_cmd = -(K_p * x_pos + K_d * x_vel);

% Calculate AMB force
F_amb = k_s * x_pos + k_i * i_actual;

% Check for current saturation
i_saturated = abs(i_cmd) > i_max;
if any(i_saturated)
    fprintf('  WARNING: Current command saturated (max = %.1f A)\n', i_max);
end

%% Rotor orbit (x-y plane)
% For this single-axis simulation, orbit is just the radial displacement
% In practice, both x and y AMBs would be active
% For visualization, assume y-direction has similar response
x_orbit = x_pos;
y_orbit = zeros(size(x_pos)); % Simplified: assume y=0 for single-axis disturbance

%% Package results
response = struct();
response.t = t;
response.x = x_pos;
response.v = x_vel;
response.i_cmd = i_cmd;
response.i = i_actual;
response.F_amb = F_amb;
response.x_orbit = x_orbit;
response.y_orbit = y_orbit;

end
