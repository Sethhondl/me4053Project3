function [E, E_usable] = flywheelEnergy(omega, I)
% flywheelEnergy - Calculate kinetic energy stored in flywheel
%
% Syntax: [E, E_usable] = flywheelEnergy(omega, I)
%
% Inputs:
%    omega - Angular velocity [rad/s]
%    I     - Mass moment of inertia [kg*m^2]
%
% Outputs:
%    E        - Total kinetic energy [J]
%    E_usable - Usable energy between 100% and 0% SoC [J]
%             (0% SoC = omega_max/2, 100% SoC = omega_max)
%
% Notes:
%    Kinetic energy: E = 0.5 * I * omega^2
%    Usable energy: E_usable = 0.5 * I * (omega_max^2 - omega_min^2)
%                             = 0.5 * I * omega_max^2 * (1 - 0.25)
%                             = 0.375 * I * omega_max^2

% Total energy
E = 0.5 * I * omega^2;

% Usable energy (assuming omega is omega_max)
% Between 100% SoC (omega_max) and 0% SoC (omega_max/2)
E_usable = 0.5 * I * (omega^2 - (omega/2)^2);
E_usable = 0.375 * I * omega^2;

end
