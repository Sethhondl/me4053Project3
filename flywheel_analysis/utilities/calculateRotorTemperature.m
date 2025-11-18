function [T_rotor, T_stator] = calculateRotorTemperature(P_rotor, P_stator, dims, omega, T_ambient)
% calculateRotorTemperature - Calculate steady-state rotor and stator temperatures
%
% Syntax: [T_rotor, T_stator] = calculateRotorTemperature(P_rotor, P_stator,
%                                                          dims, omega, T_ambient)
%
% Inputs:
%    P_rotor   - Total rotor losses [W]
%    P_stator  - Total stator losses [W]
%    dims      - Structure from calculateFlywheelDimensions
%    omega     - Angular velocity [rad/s]
%    T_ambient - Ambient/vacuum chamber temperature [°C]
%
% Outputs:
%    T_rotor  - Rotor temperature [°C]
%    T_stator - Stator temperature [°C]
%
% Notes:
%    Heat transfer in vacuum is primarily through radiation
%    Q_rad = epsilon * sigma * A * (T_hot^4 - T_cold^4)
%
%    Simplified thermal network:
%    - Rotor losses -> Rotor body -> Radiation to stator/housing
%    - Stator losses -> Stator body -> Radiation to housing
%    - Small conduction through AMB air gaps (negligible in vacuum)

% Constants
sigma = 5.67e-8;      % Stefan-Boltzmann constant [W/(m^2*K^4)]
epsilon_steel = 0.6;  % Emissivity of steel (oxidized)
epsilon_comp = 0.9;   % Emissivity of composite

% Surface areas for radiation
% Rotor surface area (flywheel dominates)
A_rotor_rad = 2 * pi * dims.r_fw_outer^2 + ...  % top and bottom faces
              2 * pi * dims.r_fw_outer * dims.l_flywheel;  % outer circumference

% Stator surface area (motor stator)
r_stator_outer = dims.r_pm_outer + 0.001 + 0.03; % air gap + stator thickness
A_stator_rad = 2 * pi * r_stator_outer * dims.l_motor;

% Iterative solution for temperatures (radiation is nonlinear)
% Assume initial temperatures
T_rotor = T_ambient + 50;  % Initial guess [°C]
T_stator = T_ambient + 30; % Initial guess [°C]

% Convert to Kelvin for radiation calculations
T_amb_K = T_ambient + 273.15;

% Iteration parameters
max_iter = 100;
tol = 0.1; % °C

for iter = 1:max_iter
    T_rotor_K = T_rotor + 273.15;
    T_stator_K = T_stator + 273.15;

    % Heat transfer from rotor to stator (radiation)
    Q_rotor_to_stator = epsilon_comp * sigma * A_rotor_rad * ...
                        (T_rotor_K^4 - T_stator_K^4);

    % Heat transfer from stator to ambient (radiation through housing)
    Q_stator_to_amb = epsilon_steel * sigma * A_stator_rad * ...
                      (T_stator_K^4 - T_amb_K^4);

    % Energy balance
    % Rotor: P_rotor = Q_rotor_to_stator
    % Stator: P_stator + Q_rotor_to_stator = Q_stator_to_amb

    % Update temperatures using Newton's method (simplified)
    % For rotor
    if Q_rotor_to_stator > 0
        T_rotor_new = T_rotor - 0.1 * (Q_rotor_to_stator - P_rotor) / ...
                      (4 * epsilon_comp * sigma * A_rotor_rad * T_rotor_K^3);
    else
        T_rotor_new = T_rotor + 10;
    end

    % For stator
    if Q_stator_to_amb > 0
        T_stator_new = T_stator - 0.1 * (Q_stator_to_amb - P_stator - Q_rotor_to_stator) / ...
                       (4 * epsilon_steel * sigma * A_stator_rad * T_stator_K^3);
    else
        T_stator_new = T_stator + 10;
    end

    % Check convergence
    if abs(T_rotor_new - T_rotor) < tol && abs(T_stator_new - T_stator) < tol
        T_rotor = T_rotor_new;
        T_stator = T_stator_new;
        break;
    end

    T_rotor = T_rotor_new;
    T_stator = T_stator_new;

    % Ensure physical temperatures
    T_rotor = max(T_ambient, T_rotor);
    T_stator = max(T_ambient, T_stator);
end

% If solution didn't converge or is unphysical, use simplified estimate
if iter >= max_iter || T_rotor > 200 || T_stator > 200
    % Simplified linearized estimate
    h_eff = 10; % Effective heat transfer coefficient [W/(m^2*K)]
    T_rotor = T_ambient + P_rotor / (h_eff * A_rotor_rad);
    T_stator = T_ambient + (P_stator + P_rotor) / (h_eff * A_stator_rad);
end

end
