% Compare inertia formulas
r_fly = 0.430/2;  % outer radius
r_shaft = 0.084/2;  % inner radius
h_fly = 1.0;
rho_fly = 1600;

% Mass (same for both)
vol_fly = pi * (r_fly^2 - r_shaft^2) * h_fly;
m_fly = rho_fly * vol_fly;

% Peyton's formula (uses minus)
J_peyton = 0.5 * m_fly * (r_fly^2 - r_shaft^2);

% Correct formula (uses plus)
J_correct = 0.5 * m_fly * (r_fly^2 + r_shaft^2);

fprintf('Flywheel mass: %.2f kg\n', m_fly);
fprintf('Peyton inertia (r^2 - r^2): %.4f kg*m^2\n', J_peyton);
fprintf('Correct inertia (r^2 + r^2): %.4f kg*m^2\n', J_correct);
fprintf('Difference: %.1f%%\n', 100*(J_correct - J_peyton)/J_correct);
