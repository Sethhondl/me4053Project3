function SoC = speed2soc(omega, omega_max)
% speed2soc - Convert angular velocity to state of charge
%
% Syntax: SoC = speed2soc(omega, omega_max)
%
% Inputs:
%    omega     - Angular velocity [rad/s]
%    omega_max - Maximum angular velocity [rad/s] (at 100% SoC)
%
% Outputs:
%    SoC - State of charge [0 to 1] (0 = 0%, 1 = 100%)
%
% Notes:
%    From soc2speed derivation:
%    omega = omega_max * sqrt(0.25 + 0.75 * SoC)
%
%    Solving for SoC:
%    (omega/omega_max)^2 = 0.25 + 0.75 * SoC
%    SoC = ((omega/omega_max)^2 - 0.25) / 0.75

omega_ratio = omega / omega_max;
SoC = (omega_ratio^2 - 0.25) / 0.75;

% Clamp to valid range [0, 1]
SoC = max(0, min(1, SoC));

end
