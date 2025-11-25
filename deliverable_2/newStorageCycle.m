function power = newStorageCycle(time)
% NEWSTORAGECYCLE New 6-hour storage cycle for design study
%
% Input:
%   time - Time in seconds [0 to 21600] (6 hours)
%
% Output:
%   power - Grid power demand [W]
%           Positive = discharge to grid
%           Negative = charge from grid
%
% This represents a demanding but feasible 6-hour frequency regulation cycle
% designed to challenge the flywheel while maintaining energy balance.
% The cycle has higher peak powers than baseline but balanced energy.

% Cycle duration: 6 hours = 21600 seconds
T_cycle = 21600;

% Normalize time to 0-1
t_norm = time / T_cycle;

% Base power amplitude - higher frequency variations than baseline
% The 6-hour cycle tests sustained operation with frequent power reversals
P_base = 70000;  % 70 kW base amplitude (vs 50 kW baseline)

% Multi-frequency components for realistic grid frequency regulation
% Higher frequencies ensure faster energy cycling (charge/discharge)
% This keeps energy swings within flywheel capacity
% Lowest frequency period should be << available energy / power
f1 = 12;    % 12 cycles over 6 hours (30-min period)
f2 = 24;    % 24 cycles (15-min period, like baseline)
f3 = 48;    % 48 cycles (~7.5-min period)
f4 = 96;    % 96 cycles (~3.75-min period)
f5 = 192;   % 192 cycles (~1.9-min period)

% Create composite signal with zero-mean sinusoids (balanced energy)
% Higher frequencies = more rapid charge/discharge = lower net energy swing
power = P_base * (...
    0.35 * sin(2*pi*f1*t_norm) + ...
    0.30 * sin(2*pi*f2*t_norm + 0.5) + ...
    0.20 * sin(2*pi*f3*t_norm + 1.2) + ...
    0.10 * sin(2*pi*f4*t_norm + 2.0) + ...
    0.05 * sin(2*pi*f5*t_norm + 0.3) ...
);

% Ensure vectorized operation
power = power(:)';
if isscalar(time)
    power = power(1);
end

end
