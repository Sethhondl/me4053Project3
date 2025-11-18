function results = simulateStorageCycle(config, t_cycle, P_cycle, SoC_initial)
% simulateStorageCycle - Simulate flywheel through a power storage cycle
%
% Syntax: results = simulateStorageCycle(config, t_cycle, P_cycle, SoC_initial)
%
% Inputs:
%    config      - Configuration structure (from baseline_config or design)
%    t_cycle     - Time vector for cycle [s]
%    P_cycle     - Power profile (+ = charge, - = discharge) [W]
%    SoC_initial - Initial state of charge [0 to 1]
%
% Outputs:
%    results - Structure containing:
%       .t_sim      - Simulation time vector [s]
%       .SoC_sim    - SoC trajectory [0 to 1]
%       .omega_sim  - Speed trajectory [rad/s]
%       .T_rotor_sim - Rotor temperature trajectory [°C]
%       .E_in       - Total energy charged [J]
%       .E_out      - Total energy discharged [J]
%       .E_selfdischarge - Energy lost to self-discharge [J]
%       .efficiency - Cycle efficiency (E_out / (E_in + E_selfdischarge))
%       .success    - True if cycle completed without violating constraints

% Extract parameters
omega_max = config.omega_max;
I_total = config.I_total;
dims = config.dims;
magnet_thickness = config.magnet_thickness;
T_ambient = config.T_ambient;
T_max = 100; % Maximum safe temperature [°C]

% Initialize simulation
dt = mean(diff(t_cycle)); % Time step [s]
n_steps = length(t_cycle);

SoC_sim = zeros(n_steps, 1);
omega_sim = zeros(n_steps, 1);
T_rotor_sim = zeros(n_steps, 1);
T_stator_sim = zeros(n_steps, 1);

% Initial conditions
SoC_sim(1) = SoC_initial;
omega_sim(1) = soc2speed(SoC_initial, omega_max);
T_rotor_sim(1) = T_ambient;
T_stator_sim(1) = T_ambient;

% Energy accounting
E_in = 0;
E_out = 0;
E_selfdischarge = 0;

% Motor/generator parameters
r_rotor = dims.r_pm_outer;
l_motor = dims.l_motor;
A_rotor = 2 * pi * r_rotor * l_motor;

% Simulation loop
success = true;
for i = 2:n_steps
    % Current state
    omega = omega_sim(i-1);
    SoC = SoC_sim(i-1);
    T_rotor = T_rotor_sim(i-1);

    % Power command for this time step
    P_cmd = P_cycle(i);

    % Calculate required stator current
    % Simplified: assume constant efficiency motor
    eta_motor = 0.95; % Motor efficiency
    if P_cmd > 0
        % Charging (motoring)
        P_mech = P_cmd * eta_motor;
    elseif P_cmd < 0
        % Discharging (generating)
        P_mech = P_cmd / eta_motor;
    else
        P_mech = 0;
    end

    % Calculate stator current based on power
    % This is simplified - real implementation would solve coupled equations
    if omega > 0
        T_mech = P_mech / omega;
    else
        T_mech = 0;
    end

    % Estimate stator current (iterative, simplified for placeholder)
    I_stator = 10; % Initial guess
    for iter = 1:5
        tau = magneticShear(magnet_thickness, I_stator);
        T_available = tau * A_rotor * r_rotor;
        if T_available > 0 && abs(T_mech) > 0
            I_stator = abs(I_stator * T_mech / T_available);
        end
    end
    I_stator = min(I_stator, 500); % Limit current

    % Calculate losses (always present, even at zero power)
    P_rotor = rotorLosses(magnet_thickness, 2*r_rotor, l_motor, I_stator, omega);
    P_stator = statorLosses(magnet_thickness, 2*r_rotor, l_motor, I_stator, omega);
    P_loss_total = P_rotor + P_stator;

    % Update energy
    % dE/dt = P_mech - P_loss
    % where E = 0.5 * I * omega^2
    dE_dt = P_mech - P_loss_total;

    E_current = 0.5 * I_total * omega^2;
    E_next = E_current + dE_dt * dt;
    E_next = max(E_next, 0); % Can't have negative energy

    % Calculate new omega
    omega_next = sqrt(2 * E_next / I_total);

    % Convert to SoC
    SoC_next = speed2soc(omega_next, omega_max);

    % Check constraints
    if SoC_next < 0
        success = false;
        fprintf('  WARNING: SoC dropped below 0%% at t=%.1f s\n', t_cycle(i));
        SoC_next = 0;
        omega_next = soc2speed(0, omega_max);
    end

    % Update temperature (simplified quasi-static)
    [T_rotor_next, T_stator_next] = calculateRotorTemperature(P_rotor, P_stator, ...
                                                               dims, omega_next, T_ambient);

    if T_rotor_next > T_max
        fprintf('  WARNING: Rotor temperature exceeded %.0f°C at t=%.1f s\n', T_max, t_cycle(i));
    end

    % Store results
    omega_sim(i) = omega_next;
    SoC_sim(i) = SoC_next;
    T_rotor_sim(i) = T_rotor_next;
    T_stator_sim(i) = T_stator_next;

    % Energy accounting
    if P_cmd > 0
        E_in = E_in + P_cmd * dt;
    elseif P_cmd < 0
        E_out = E_out + abs(P_cmd) * dt;
    end
    E_selfdischarge = E_selfdischarge + P_loss_total * dt;
end

% Calculate efficiency
% Efficiency = E_out / (E_in + E_selfdischarge_recovery)
% The self-discharge energy must be recovered, so it counts as input
efficiency = E_out / (E_in + E_selfdischarge);

% Package results
results = struct();
results.t_sim = t_cycle;
results.SoC_sim = SoC_sim;
results.omega_sim = omega_sim;
results.T_rotor_sim = T_rotor_sim;
results.T_stator_sim = T_stator_sim;
results.E_in = E_in;
results.E_out = E_out;
results.E_selfdischarge = E_selfdischarge;
results.efficiency = efficiency;
results.success = success;

end
