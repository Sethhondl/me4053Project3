% Experiment: What F_rad needed for T_max < 100°C?

% Constants
sigma = 5.67e-8;           % Stefan-Boltzmann
T_housing_K = 30 + 273.15; % 303.15 K
T_max_target = 100 + 273.15; % 373.15 K (target)

% Geometry (from v2)
r_outer = 0.430 / 2;
L_fly = 1.0;
A_rotor = 2*pi*r_outer*L_fly + 2*pi*r_outer^2;

% Max rotor losses (from v2 results - occurs at 100% SoC)
Q_rotor_max = 420; % W (approximately)

% Current F_rad
epsilon_rotor = 0.4;
epsilon_housing = 0.9;
housing_clearance = 0.020;
housing_inner_dia = 0.430 + 2*housing_clearance;
A_housing = 2*pi*(housing_inner_dia/2)*L_fly + 2*pi*(housing_inner_dia/2)^2;
F_rad_current = 1/epsilon_rotor + (A_rotor/A_housing)*(1/epsilon_housing - 1);

% Current max temperature
T_current = (T_housing_K^4 + Q_rotor_max*F_rad_current/(sigma*A_rotor))^0.25;

fprintf('=== CURRENT SYSTEM ===\n');
fprintf('Rotor surface area: %.3f m^2\n', A_rotor);
fprintf('Max rotor losses: %.0f W\n', Q_rotor_max);
fprintf('Current F_rad: %.2f\n', F_rad_current);
fprintf('Current max temp: %.1f C\n\n', T_current - 273.15);

% What F_rad needed for T = 100°C?
% T_target^4 = T_housing^4 + Q * F_rad_needed / (sigma * A)
% F_rad_needed = (T_target^4 - T_housing^4) * sigma * A / Q

F_rad_needed = (T_max_target^4 - T_housing_K^4) * sigma * A_rotor / Q_rotor_max;

fprintf('=== TO GET T_max = 100 C ===\n');
fprintf('F_rad needed: %.2f\n', F_rad_needed);
fprintf('Reduction from current: %.1f%%\n\n', 100*(F_rad_current - F_rad_needed)/F_rad_current);

% What emissivity would give this F_rad? (assuming same geometry)
% F_rad = 1/epsilon_rotor + (A_rotor/A_housing)*(1/epsilon_housing - 1)
% Solve for epsilon_rotor:
% 1/epsilon_rotor = F_rad - (A_rotor/A_housing)*(1/epsilon_housing - 1)
ratio = A_rotor/A_housing;
term2 = ratio * (1/epsilon_housing - 1);
epsilon_rotor_needed = 1 / (F_rad_needed - term2);

fprintf('=== EQUIVALENT ROTOR EMISSIVITY ===\n');
fprintf('Current rotor emissivity: %.2f\n', epsilon_rotor);
fprintf('Needed rotor emissivity: %.2f\n', epsilon_rotor_needed);

% Sweep F_rad values
fprintf('\n=== TEMPERATURE vs F_rad ===\n');
fprintf('F_rad    T_max [C]\n');
fprintf('-----    ---------\n');
for F = [1.0, 1.5, 2.0, 2.2, 2.4, 2.6, 2.8, 3.0]
    T = (T_housing_K^4 + Q_rotor_max*F/(sigma*A_rotor))^0.25 - 273.15;
    marker = '';
    if abs(F - F_rad_current) < 0.05
        marker = ' <-- current';
    elseif abs(F - F_rad_needed) < 0.05
        marker = ' <-- needed for 100C';
    end
    fprintf('%.1f      %.1f%s\n', F, T, marker);
end
