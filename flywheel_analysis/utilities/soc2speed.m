function omega = soc2speed(SoC, omega_max)
% soc2speed - Convert state of charge to angular velocity
%
% Syntax: omega = soc2speed(SoC, omega_max)
%
% Inputs:
%    SoC       - State of charge [0 to 1] (0 = 0%, 1 = 100%)
%    omega_max - Maximum angular velocity [rad/s] (at 100% SoC)
%
% Outputs:
%    omega - Angular velocity [rad/s]
%
% Notes:
%    Energy relationship: E = 0.5 * I * omega^2
%    At 0% SoC: omega = omega_max / 2
%    At 100% SoC: omega = omega_max
%
%    Energy at 100%: E_100 = 0.5 * I * omega_max^2
%    Energy at 0%:   E_0   = 0.5 * I * (omega_max/2)^2 = 0.125 * I * omega_max^2
%    Usable energy: E_usable = E_100 - E_0 = 0.375 * I * omega_max^2
%
%    At SoC:
%    E = E_0 + SoC * E_usable
%    0.5 * I * omega^2 = 0.125 * I * omega_max^2 + SoC * 0.375 * I * omega_max^2
%    omega^2 = omega_max^2 * (0.125 + 0.375 * SoC)
%    omega = omega_max * sqrt(0.25 + 0.75 * SoC)

omega = omega_max * sqrt(0.25 + 0.75 * SoC);

end
