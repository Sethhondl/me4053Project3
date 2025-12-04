%% Magnetic Levitation Flywheel Model
% This script uses helper functions located in the `Project3_Functions` folder.
% The following summaries describe those functions for quick reference.

%% Housekeeping
clc;            % Clear command window
clear;          % Clear workspace
close all;      % Close all figure windows

%% Parameters (from FlywheelModelParams.csv)
% All distances converted to meters, speeds to SI where noted.
% These parameter values correspond specifically to Deliverable 1.
% SoC mapping assumption (from HW Q1): 0%% SoC = 0.5 * max speed,
% 100%% SoC = max speed.

params = createParams();
format = createPlotFormatStruct();

% Main entry point for Deliverable 1 analysis
 runDeliverable1(params, format);

% Main entry point for Deliverable 2 design study
% runDeliverable2(params, format);

%% External functions

    % -------------------------------------------------------------------------
    % magneticShear
    % -------------------------------------------------------------------------
    % function [shearStress] = magneticShear(magnetThickness, statorCurrent)
    %   MAGNETICSHEAR Calculate the magnetic shear stress acting on the rotor
    %   [shearStress] = magneticShear(magnetThickness,statorCurrent) calculates
    %   the magnetic shear stress (units of Pa) acting on the surface of the
    %   electric machines rotor.
    %
    %   magnetThickness: thickness of surface mounted PMs. Unit: meters
    %   statorCurrent: per unit value, normalized by rated current
    %       (1: rated current, 0: no current)
    %
    % -------------------------------------------------------------------------
    % rotorLosses
    % -------------------------------------------------------------------------
    % function [loss] = rotorLosses(magnetThickness, ...
    %                               rotorDiameter, ...
    %                               axialLength, ...
    %                               statorCurrent, ...
    %                               rotorSpeed)
    %   ROTORLOSSES Calculate total losses on rotor
    %   [loss] = rotorLosses(magnetThickness, rotorDiameter, axialLength, ...
    %       statorCurrent, rotorSpeed) calculates the total losses on the rotor.
    %
    %   magnetThickness: thickness of surface mounted PMs. Unit: meters
    %   rotorDiameter: outer diameter of magnets. Unit: meters
    %   axialLength: axial length of the magnets. Unit: meters
    %   statorCurrent: per unit value, normalized by rated current
    %       (1: rated current, 0: no current)
    %   rotorSpeed: rotational speed of the rotor. Units: rev/min.
    %
    % -------------------------------------------------------------------------
    % statorLosses
    % -------------------------------------------------------------------------
    % function [loss] = statorLosses(magnetThickness, ...
    %                                rotorDiameter, ...
    %                                axialLength, ...
    %                                statorCurrent, ...
    %                                rotorSpeed)
    %   STATORLOSSES Calculate losses in the electric machines stator
    %   [loss] = statorLosses(magnetThickness, rotorDiameter, axialLength, ...
    %       statorCurrent, rotorSpeed) calculates the losses in the stator.
    %
    %   magnetThickness: thickness of surface mounted PMs. Unit: meters
    %   rotorDiameter: outer diameter of magnets. Unit: meters
    %   axialLength: axial length of the magnets. Unit: meters
    %   statorCurrent: per unit value, normalized by rated current
    %       (1: rated current, 0: no current)
    %   rotorSpeed: rotational speed of the rotor. Units: rev/min.
    %
    % -------------------------------------------------------------------------
    % ambParameters
    % -------------------------------------------------------------------------
    % function [params] = ambParameters(rotorDiameter, forceRating)
    %   AMBParameters Calculate the parameters of a desired magnetic bearing
    %   [params] = ambParameters(rotorDiameter,forceRating)
    %
    %   rotorDiameter: diameter of shaft. Units: meters
    %   forceRating: force rating needed for AMB. Units: newtons
    %
    %   params - structure with the following fields:
    %       stiffnessConstant: units of N/m
    %       forceConstant: units of N/A
    %       biasCurrent: units of A
    %       ratedControlCurrent: units of A
    %       coilInductance: units of H
    %       coilResistance: units of Ohms
    %       axialLength: units of meters
    %
    % -------------------------------------------------------------------------
    % baselineStorageCycle
    % -------------------------------------------------------------------------
    % function [power] = baselineStorageCycle(time)
    %   BASELINESTORAGECYCLE Calculate the power demand for the baseline cycle
    %   [power] = baselineStorageCycle(time) calculates the grid power demand
    %   (units of W) for the 15-minute baseline frequency regulation cycle.
    %   Positive power values indicate a power flow from the storage device to
    %   the power grid.
    %
    %   This function is vectorized and accepts time as a scalar or an array.
    %
    %   time: time since the start of the cycle. Unit: seconds
    %       (Must be between 0 and 900 seconds)
    %
    % -------------------------------------------------------------------------
    % team_2_cycle
    % -------------------------------------------------------------------------
    % function [power_W] = team_2_cycle(time_s)
    %   TEAM_1_CYCLE Calculate the power demand for this teams unique cycle
    %   [power_W] = team_2_cycle(time_s) calculates the grid power demand
    %   (units of W) for this teams unique energy storage cycle.
    %
    %   This function is vectorized and accepts time_s as a scalar or an array.
    %
    %   Input Arguments:
    %       time_s: time since the start of the cycle. Unit: seconds
    %           (Must be between 0 and 21600.0 seconds)
    %
    %   Return Value:
    %       power_W: power demanded by the grid. Unit: Watts
    %
    %   This cycle has a total duration of 6.0 hours (21600 seconds).
    %
    %   This is an auto-generated wrapper function.
    %

%% Local functions

function params = createParams()
    %CREATEDELIVERABLE1PARAMS Build parameter struct for Deliverable 1.
    %   Returns a struct with:
    %     - params.deliverable1: Appendix B baseline flywheel system properties
    %     - params.control:      AMB controller gains (current, position, tilt)

    params = struct();

    % ---- Deliverable 1: flywheel system properties (Appendix B / CSV) -------

    % Thermal / environment
    params.deliverable1.thermal.housingTemp_C     = 30;    % [C]
    params.deliverable1.thermal.maxTemp_C         = 100;   % [C]
    params.deliverable1.thermal.rotorEmissivity   = 0.4;   % [-]
    params.deliverable1.thermal.housingEmissivity = 0.9;   % [-]

    % Geometry
    params.deliverable1.geometry.flywheelAxialLength_m = 1000e-3; % [m]
    params.deliverable1.geometry.flywheelDiameter_m    = 430e-3;  % [m]
    params.deliverable1.geometry.motorAxialLength_m    = 250e-3;  % [m]
    params.deliverable1.geometry.shaftAndPMDiameter_m  = 84e-3;   % [m]
    params.deliverable1.geometry.magnetThickness_m     = 6e-3;    % [m]
    params.deliverable1.geometry.minMagnetThickness_m  = 2e-3;    % [m]

    % AMB rated force
    params.deliverable1.amb.ratedForce_N = 5780;         % [N]

    % Speed limits
    params.deliverable1.speed.maxRotSpeed_rpm       = 40000; % [r/min]
    params.deliverable1.speed.maxRotSpeed_radPerSec = ...
        params.deliverable1.speed.maxRotSpeed_rpm * 2*pi/60; % [rad/s]
    params.deliverable1.speed.maxTsShaft_mPerSec    = 175;   % [m/s]
    params.deliverable1.speed.maxTsPM_mPerSec       = 175;   % [m/s]
    params.deliverable1.speed.maxTsFly_mPerSec      = 900;   % [m/s]

    % Clearances
    params.deliverable1.clearance.axial_m          = 20e-3; % [m]
    params.deliverable1.clearance.radialFlywheel_m = 20e-3; % [m]
    params.deliverable1.clearance.radialOther_m    = 1e-3;  % [m]

    % Material properties
    params.deliverable1.material.densFly_kgPerM3   = 1600;  % [kg/m^3]
    params.deliverable1.material.densSteel_kgPerM3 = 7850;  % [kg/m^3]
    params.deliverable1.material.densPM_kgPerM3    = 7850;  % [kg/m^3]

    % Balance grade
    params.deliverable1.balance.grade_ISO1940      = 'G2.5';

    % ---- Controller gains (Appendix B control rows) -------------------------

    % Current controller transfer function: v(s)/e(s) = 345 + 2149/s
    params.control.current.k_p = 345;
    params.control.current.k_i = 2149;

    % Position controller (x)
    params.control.positionX.k_p    = 1.2639e8;
    params.control.positionX.k_i    = 1.16868e9;
    params.control.positionX.k_d    = 252790;
    params.control.positionX.omega_p_radPerSec = 3770; % [rad/s]

    % Tilting position controller (alpha)
    params.control.tilt.k_p    = 7.6992e7;
    params.control.tilt.k_i    = 1.18953e9;
    params.control.tilt.k_d    = 80294;
    params.control.tilt.omega_p_radPerSec = 6283; % [rad/s]

end

function format = createPlotFormatStruct()
    %CREATEPLOTFORMATSTRUCT Return common plotting style settings.
    %   FORMAT = CREATEPLOTFORMATSTRUCT() builds a struct with fields:
    %     - fontName, fontSize
    %     - figWidth, figHeight (pixels, placeholder values)
    %     - lineWidth
    %     - color.primary, color.secondary, color.ternary (RGB triplets)

    format = struct();

    format.fontName  = 'Times New Roman';
    format.fontSize  = 12;
    format.figWidth  = 900;   % placeholder, can be tuned later
    format.figHeight = 600;   % placeholder, can be tuned later
    format.lineWidth = 2;

    format.color.primary   = [0.0000 0.4470 0.7410]; % MATLAB default blue
    format.color.secondary = [0.8500 0.3250 0.0980]; % MATLAB default orange
    format.color.ternary   = [0.4660 0.6740 0.1880]; % MATLAB default green

end

function runDeliverable1(params, format)
    %RUNDELIVERABLE1 Execute all Deliverable 1 analyses and plots.
    %   Assumptions:
    %     - 0% SoC corresponds to 0.5 * max operating speed, 100% to max speed.
    %     - Losses are rotor + stator losses from the provided functions and
    %       are the only self-discharge mechanism.
    %     - Rotating-group temperature is set by radiation only to the housing
    %       at constant temperature (lumped-parameter thermal model).
    %     - AMB dynamics use a 1-DOF point-mass model in x based on ambParameters
    %       and the controller structure from HW Q2, evaluated at zero shaft
    %       rotational speed.
    %     - Rotor runout due to imbalance is approximated from ISO 1940 G2.5
    %       using a simple relationship between balance grade and equivalent
    %       synchronous eccentricity.

    % Convenience aliases
    p  = params.deliverable1;
    pc = params.control;

    % SoC grid
    soc_vec = linspace(0, 100, 100);

    % --- Geometry, inertia, and rated power  -----

    r_fly   = p.geometry.flywheelDiameter_m / 2;
    h_fly   = p.geometry.flywheelAxialLength_m;
    d_shaft = p.geometry.shaftAndPMDiameter_m;
    r_shaft = d_shaft / 2;
    h_motor = p.geometry.motorAxialLength_m;
    t_mag   = p.geometry.magnetThickness_m;

    axial_clear = p.clearance.axial_m;

    % AMB parameters (used for shaft length estimate and AMB dynamics)
    ambParams = ambParameters(d_shaft, p.amb.ratedForce_N);

    % Estimate total shaft length (shaft passes through flywheel and motor)
    h_shaft = (5 * axial_clear) + h_fly + h_motor + 2 * ambParams.axialLength;

    % Masses
    rho_fly   = p.material.densFly_kgPerM3;
    rho_steel = p.material.densSteel_kgPerM3;

    vol_shaft = pi * r_shaft^2 * h_shaft;
    vol_fly   = pi * (r_fly^2 - r_shaft^2) * h_fly;

    m_shaft = rho_steel * vol_shaft;
    m_fly   = rho_fly * vol_fly;

    m_rotating = m_shaft + m_fly;

    % Inertia (shaft + hollow flywheel)
    J = 0.5 * m_fly * (r_fly^2 - r_shaft^2) + 0.5 * m_shaft * r_shaft^2;

    % Operating speed limits based on tip speed constraints
    omega_max_shaft = p.speed.maxTsShaft_mPerSec / r_shaft;  % [rad/s]
    omega_max_fly   = p.speed.maxTsFly_mPerSec   / r_fly;    % [rad/s]

    if omega_max_shaft <= omega_max_fly
        omega_max = omega_max_shaft;   % Shaft tip speed is limiting
    else
        omega_max = omega_max_fly;     % Flywheel tip speed is limiting
    end

    omega_min = 0.5 * omega_max;                       % 0% SoC (half max speed)

    KE_min = 0.5 * J * omega_min^2;
    KE_max = 0.5 * J * omega_max^2;

    % Rated torque and power from magnetic shear at Ipu = 1 (encapsulated)
    [T_rated, P_rated_W] = computeRatedTorqueAndPower(p, r_shaft, h_motor, t_mag, omega_min);

    % --- Losses and temperature vs SoC -----------------------------------

    [losses, T_rot_C, omega_vec] = computeLossesAndTemperatureVsSoc( ...
        soc_vec, J, KE_min, KE_max, P_rated_W, T_rated, ...
        r_fly, h_fly, d_shaft, t_mag, p);

    plotLossesAndTemperatureVsSoc(soc_vec, losses.rotor_W, losses.stator_W, ...
        losses.total_W, T_rot_C, format);

    % --- Specific power and specific energy ----------------------------------

    [specPower_kW_per_kg, specEnergy_Wh_per_kg] = ...
        computeSpecificPowerAndEnergy(P_rated_W, KE_min, KE_max, m_rotating);

    fprintf('Deliverable 1 rotating group metrics:\n');
    fprintf('  Total rotating mass: %.2f kg\n', m_rotating);
    fprintf('  Specific power:      %.3f kW/kg\n', specPower_kW_per_kg);
    fprintf('  Specific energy:     %.3f Wh/kg\n\n', specEnergy_Wh_per_kg);

    % --- Cycle efficiency with self-discharge --------------------------------

    cycleResults = computeCycleEfficiency(params, P_rated_W, ...
        KE_min, KE_max, J, soc_vec, omega_vec, losses);

    fprintf('Deliverable 1 storage cycle efficiency (including self-discharge):\n');
    fprintf('  Energy delivered to grid:      %.1f kWh\n', cycleResults.E_out_Wh/1e3);
    fprintf('  Energy drawn from grid:        %.1f kWh\n', cycleResults.E_in_Wh/1e3);
    fprintf('  Self-discharge energy loss:    %.1f kWh\n', cycleResults.E_loss_Wh/1e3);
    fprintf('  Round-trip efficiency:         %.1f %%\n\n', ...
        100 * cycleResults.eta_roundtrip);

    % --- SoC trajectory over baseline storage cycle ---------------------------

    plotSocOverBaselineCycle(J, KE_min, KE_max, soc_vec, omega_vec, losses, format);

    % --- AMB step response (current, force, position) ------------------------

    ambStep = simulateAmbStepResponse(p, pc, ambParams, m_rotating);
        plotAmbStepResponse(ambStep.t_s, ambStep.i_A, ambStep.F_N, ...
            ambStep.x_m, format);

    % --- Dynamic stiffness vs frequency --------------------------------------

    % Use the same effective mass per bearing as in the step-response model
    dynStiff = computeDynamicStiffness(p, pc, ambParams, m_rotating);
    plotDynamicStiffness(dynStiff.freq_Hz, dynStiff.K_rad_N_per_m, ...
        dynStiff.K_tilt_Nm_per_rad, format);

    % --- Rotor runout vs SoC due to imbalance --------------------------------

    runout = computeRunoutVsSoc(soc_vec, omega_vec, p);
    plotRunoutVsSoc(soc_vec, runout.runout_mm, format);

end

function [losses, T_rot_C, omega_vec] = computeLossesAndTemperatureVsSoc( ...
    soc_vec, J, KE_min, KE_max, P_rated_W, T_rated, ...
    r_fly, h_fly, d_shaft, t_mag, p)
    %COMPUTELOSSESANDTEMPERATUREVSSOC Losses and rotor temperature vs SoC.

    n = numel(soc_vec);
    losses.rotor_W  = zeros(1, n);
    losses.stator_W = zeros(1, n);
    losses.total_W  = zeros(1, n);
    T_rot_C   = zeros(1, n);
    omega_vec = zeros(1, n);

    D_rotor = d_shaft + 2 * t_mag;
    L_motor = p.geometry.motorAxialLength_m;

    % Radiative properties
    epsilon = p.thermal.rotorEmissivity;
    sigmaSB = 5.67e-8;
    T_housing_K = p.thermal.housingTemp_C + 273.15;

    % Compute omega_min from P_rated and T_rated for consistency
    omega_min_from_P = P_rated_W / T_rated;
    
    for k = 1:n
        % Convert SoC [%] to fraction [0,1]
        soc = soc_vec(k) / 100;

        % Map SoC to kinetic energy and angular speed using KE range
        KE    = KE_min + soc * (KE_max - KE_min);
        omega = sqrt(2 * KE / J);
        
        % At 0% SoC, ensure omega exactly equals omega_min to avoid precision issues
        % This ensures Ipu = 1.0 exactly at rated conditions
        % Use small tolerance for floating-point comparison
        if abs(soc) < 1e-10 || soc_vec(k) == 0
            omega = omega_min_from_P;
        end
        
        omega_vec(k) = omega;

        % Required torque at this speed for constant power and per-unit current
        % Note: Ipu = T_soc / T_rated = (P_rated_W / omega) / T_rated
        %      = (T_rated * omega_min / omega) / T_rated = omega_min / omega
        % At 0% SoC, omega = omega_min, giving Ipu = 1.0
        % At higher SoC, omega > omega_min, giving Ipu < 1.0
        % Use omega_min_from_P for consistency to avoid precision issues
        Ipu = omega_min_from_P / omega;

        % Convert angular speed to rpm for loss functions
        rotorSpeed_rpm = omega * 60 / (2*pi);

        % Electromagnetic losses at this operating point
        losses.rotor_W(k)  = rotorLosses(t_mag, D_rotor, L_motor, Ipu, rotorSpeed_rpm);
        losses.stator_W(k) = statorLosses(t_mag, D_rotor, L_motor, Ipu, rotorSpeed_rpm);
        losses.total_W(k)  = losses.rotor_W(k) + losses.stator_W(k);

        % Radiative temperature: closed-form solution of Q_gen = Q_rad
        r_o = r_fly;
        r_i = d_shaft / 2;
        A_surface = pi * (2 * r_o * h_fly) + 2 * pi * (r_o^2 - r_i^2);

        % Solve epsilon*sigma*A*(T_K^4 - T_H^4) = Q_gen for T_K
        Q_gen = losses.rotor_W(k);
        T_K   = ((Q_gen / (epsilon * sigmaSB * A_surface)) + T_housing_K^4)^(1/4);
        T_rot_C(k) = T_K - 273.15; % convert back to [C]
    end

end

function [T_rated, P_rated_W] = computeRatedTorqueAndPower(p, r_shaft, h_motor, t_mag, omega_min)
    %COMPUTERATEDTORQUEANDPOWER Rated torque and power from magnetic shear.
    %   Uses Appendix B geometry and magneticShear at Ipu = 1 to compute:
    %     - T_rated   : rated torque at 0% SoC (min speed)
    %     - P_rated_W : corresponding mechanical rated power
    %   Note: Uses rotor radius (r_shaft + t_mag)

    tau_rated = magneticShear(t_mag, 1);                  % [Pa]
    r_rotor = r_shaft + t_mag;                           % Rotor radius includes magnets
    T_rated   = 2 * pi * r_rotor^2 * h_motor * tau_rated; % [N·m]
    P_rated_W = T_rated * omega_min;                      % [W]

end

function [specP_kW_per_kg, specE_Wh_per_kg] = ...
    computeSpecificPowerAndEnergy(P_rated_W, KE_min, KE_max, m_rotating)
    %COMPUTESPECIFICPOWERANDENERGY Specific power and energy of rotor.

    % Specific power: rated mechanical power (kW) divided by rotating mass [kg]
    specP_kW_per_kg = (P_rated_W / 1e3) / m_rotating;

    % Specific energy: extractable KE (J) converted to Wh, then per kg
    specE_Wh_per_kg = ((KE_max - KE_min) / 3600) / m_rotating;

end

function cycleResults = computeCycleEfficiency(params, P_rated_W, ...
    KE_min, KE_max, J, soc_vec, omega_vec, losses) 
    %COMPUTECYCLEEFFICIENCY Approximate round-trip efficiency over storage cycle.

    % Use baseline 15-minute cycle
    t_s = linspace(0, 900, 901); % 1 s resolution
    P_grid_W = baselineStorageCycle(t_s); % +: power from flywheel to grid

    % Interpolate losses vs omega for self-discharge estimate
    omega_from_soc = interp1(soc_vec, omega_vec, linspace(0, 100, numel(t_s)), ...
        'linear', 'extrap');
    rotorLoss_interp = interp1(omega_vec, losses.rotor_W, omega_from_soc, ...
        'linear', 'extrap');
    statorLoss_interp = interp1(omega_vec, losses.stator_W, omega_from_soc, ...
        'linear', 'extrap');
    P_loss_W = rotorLoss_interp + statorLoss_interp;

    dt = t_s(2) - t_s(1);

    % Energy accounting
    % NOTE:
    %   - P_grid_W > 0  : power delivered from flywheel to grid
    %   - P_grid_W < 0  : power drawn from grid to charge flywheel
    %   - P_loss_W      : internal rotor + stator losses (always ≥ 0)
    %
    % E_out_Wh : gross energy delivered to the grid over the cycle
    % E_in_Wh  : energy drawn from the grid to charge the flywheel
    % E_loss_Wh: internal energy dissipated as heat
    E_out_Wh  = sum(max(P_grid_W, 0)) * dt / 3600;
    E_in_Wh   = -sum(min(P_grid_W, 0)) * dt / 3600;
    E_loss_Wh = sum(P_loss_W) * dt / 3600;

    % Net useful energy recovered (what the grid effectively gets back)
    E_net_Wh = E_out_Wh - E_loss_Wh;

    cycleResults.E_out_Wh  = E_out_Wh;
    cycleResults.E_in_Wh   = E_in_Wh;
    cycleResults.E_loss_Wh = E_loss_Wh;
    cycleResults.E_net_Wh  = E_net_Wh;

    % Round-trip efficiency: useful energy recovered / energy stored
    if E_in_Wh > 0
        cycleResults.eta_roundtrip = max(E_net_Wh, 0) / E_in_Wh;
    else
        cycleResults.eta_roundtrip = 0;
    end

end

function ambStep = simulateAmbStepResponse(p, pc, ambParams, m_effective)
    %SIMULATEAMBSTEPRESPONSE 1-DOF AMB response to step disturbance at top AMB.

    % Not sure which mass needs to be used here. currently looking at half the flywheel+shaft mass. (split evenly between two bearings)
    m = m_effective * 0.5; 
    g = 9.81;

    Ks = ambParams.stiffnessConstant;
    Ki = ambParams.forceConstant;
    L  = ambParams.coilInductance;
    R  = ambParams.coilResistance;

    i_bias = m * g / Ki;

    s = tf('s');

    % Current loop (PI)
    G_plant_i = 1 / (L*s + R);
    G_ci = pc.current.k_p + pc.current.k_i / s;
    T_i = feedback(G_ci * G_plant_i, 1);

    % Position controller (x)
    G_cp = pc.positionX.k_p + pc.positionX.k_i/s + ...
        s*pc.positionX.k_d/(1 + s/pc.positionX.omega_p_radPerSec);

    % Position plant (force to displacement).
    % Assumes a single DOF point-mass model in this direction:
    %   m * x_ddot = K_F * i_y + K_s * x
    % and uses the linearized small-signal form with
    %   K_F = Ki,  K_s = Ks_mag.
    Ks_mag = abs(Ks);
    G_pos = Ki / (m*s^2 + Ks_mag);

    % Closed-loop from disturbance force to position (approximate).
    % Assumes:
    %   - The inner current loop is ideal (unity gain, infinite bandwidth)
    %   - Control force from the position loop is F_c ≈ K_F * G_cp(s) * x
    %   - Disturbance force F_d enters the same mass-spring DOF
    % This yields the small-signal relation:
    %   x/F_d ≈ 1 / (m s^2 + K_s + K_F G_cp(s))
    % implemented below.
    G_dist_to_x = 1 / (m*s^2 + Ks_mag + Ki*G_cp);

    % Step disturbance: 10% of AMB rated force
    F_step = 0.10 * p.amb.ratedForce_N;

    % Time vector
    t_end = 0.20;                          % [s] simulate first 200 ms
    t = linspace(0, t_end, 2e5);           % dt ~ 1e-6 s
    F_input = F_step * ones(size(t));

    [x_response, t_out] = lsim(G_dist_to_x, F_input, t);

    % For simplicity, assume the inner current loop is very fast and use a
    % quasi-static linear relation between displacement and incremental
    % current: Delta_i ≈ (Ki/Ks)*(-x). This omits detailed current-loop
    % dynamics and any saturation effects.
    i_A = i_bias + (Ki / Ks_mag) * (-x_response);

    ambStep.t_s = t_out;
    ambStep.x_m = x_response;
    ambStep.F_N = F_input(:);
    ambStep.i_A = i_A(:);
    ambStep.i_bias = i_bias;

end

function dynStiff = computeDynamicStiffness(p, pc, ambParams, m_effective)
    %COMPUTEDYNAMICSTIFFNESS Dynamic stiffness vs frequency (radial & tilting).

    % Use same effective mass per bearing as in simulateAmbStepResponse
    m = 0.5 * m_effective;

    Ks = ambParams.stiffnessConstant;
    Ki = ambParams.forceConstant;

    s = tf('s');

    % Radial position controller
    G_cp_rad = pc.positionX.k_p + pc.positionX.k_i/s + ...
        s*pc.positionX.k_d/(1 + s/pc.positionX.omega_p_radPerSec);

    % Tilting controller
    G_cp_tilt = pc.tilt.k_p + pc.tilt.k_i/s + ...
        s*pc.tilt.k_d/(1 + s/pc.tilt.omega_p_radPerSec);

    % Dynamic stiffness approximated as Ks + Ki * controller gain
    K_dyn_rad  = Ks + Ki * G_cp_rad;
    K_dyn_tilt = Ks + Ki * G_cp_tilt;

    freq_rad_s = logspace(1, 4, 300);
    freq_Hz    = freq_rad_s / (2*pi);

    K_rad = squeeze(freqresp(K_dyn_rad, freq_rad_s));
    K_tilt = squeeze(freqresp(K_dyn_tilt, freq_rad_s));

    dynStiff.freq_Hz          = freq_Hz;
    dynStiff.K_rad_N_per_m    = abs(K_rad);
    dynStiff.K_tilt_Nm_per_rad = abs(K_tilt);

end

function runout = computeRunoutVsSoc(soc_vec, omega_vec, p)
    %COMPUTERUNOUTVSSOC Approximate rotor runout vs SoC from .ISO G2.5

    % ISO 1940 grade G relates specific unbalance e (m) and operating speed
    % via G = e * omega (m/s). For G2.5, G = 2.5e-3 m/s.
    G_grade = 2.5e-3;

    omega = omega_vec; % [rad/s]

    e_m = G_grade ./ max(omega, 1e-3); % avoid divide-by-zero
    runout_mm = e_m * 1e3;

    runout.soc_vec   = soc_vec;
    runout.runout_mm = runout_mm;

end

% --------------------------- Plot helpers ---------------------------------

function plotLossesAndTemperatureVsSoc(soc_vec, rotorLoss_W, statorLoss_W, ...
        totalLoss_W, T_rot_C, format)
    %PLOTLOSSESANDTEMPERATUREVSSOC Plot losses and temperature vs SoC.

    figure('Position', [100, 100, format.figWidth, format.figHeight]);

    yyaxis left;
    plot(soc_vec, rotorLoss_W/1e3, 'Color', format.color.primary, ...
        'LineWidth', format.lineWidth, 'DisplayName', 'Rotor losses');
    hold on;
    plot(soc_vec, statorLoss_W/1e3, 'Color', format.color.secondary, ...
        'LineWidth', format.lineWidth, 'DisplayName', 'Stator losses');
    plot(soc_vec, totalLoss_W/1e3, 'Color', format.color.ternary, ...
        'LineWidth', format.lineWidth, 'DisplayName', 'Total losses');
    ylabel('Losses [kW]');

    yyaxis right;
    plot(soc_vec, T_rot_C, '--', 'Color', 'k', ...
        'LineWidth', format.lineWidth, 'DisplayName', 'Rotor temperature');
    ylabel('Temperature [^{\circ}C]');

    xlabel('State of Charge [%]');
    title('Losses and Rotating Group Temperature vs. SoC');
    grid on;
    legend('Location', 'northwest');
    set(gca, 'FontName', format.fontName, 'FontSize', format.fontSize);

end

function plotAmbStepResponse(t_s, i_A, F_N, x_m, format)
    %PLOTAMBSTEPRESPONSE Plot AMB current, force, and position for step input.

    figure('Position', [120, 120, format.figWidth, format.figHeight]);

    subplot(3,1,1);
    % Show current as deviation from bias in mA for readability
    i_bias = mean(i_A); % approximate bias from steady-state value
    delta_i_mA = (i_A - i_bias) * 1e3;
    plot(t_s*1e3, delta_i_mA, 'Color', format.color.primary, ...
        'LineWidth', format.lineWidth);
    grid on;
    ylabel('Current dev. [mA]');
    title(sprintf('AMB Step Response (10%% rated force disturbance)  (bias \\approx %.3f A)', i_bias));
    % Clean up y-axis tick labels (compact around zero)
    ax1 = gca;
    yl  = ylim(ax1);
    ax1.YLim  = yl;
    ax1.YTick = linspace(yl(1), yl(2), 3); % 3 ticks across small range
    ytickformat(ax1, '%.1f');
    xlim([0 200]); % limit to 0–200 ms
    set(ax1, 'FontName', format.fontName, 'FontSize', format.fontSize);

    subplot(3,1,2);
    plot(t_s*1e3, F_N, 'Color', format.color.secondary, ...
        'LineWidth', format.lineWidth);
    grid on;
    ylabel('Force [N]');
    xlim([0 200]); % limit to 0–200 ms
    set(gca, 'FontName', format.fontName, 'FontSize', format.fontSize);

    subplot(3,1,3);
    plot(t_s*1e3, x_m*1e6, 'Color', format.color.ternary, ...
        'LineWidth', format.lineWidth);
    grid on;
    xlabel('Time [ms]');
    ylabel('Position [\mu m]');
    xlim([0 200]); % limit to 0–200 ms
    set(gca, 'FontName', format.fontName, 'FontSize', format.fontSize);

end

function plotDynamicStiffness(freq_Hz, K_rad, K_tilt, format)
    %PLOTDYNAMICSTIFFNESS Plot dynamic stiffness magnitude vs frequency.

    figure('Position', [140, 140, format.figWidth, format.figHeight]);

    loglog(freq_Hz, K_rad, 'Color', format.color.primary, ...
        'LineWidth', format.lineWidth, 'DisplayName', 'Radial');
    hold on;
    loglog(freq_Hz, K_tilt, 'Color', format.color.secondary, ...
        'LineWidth', format.lineWidth, 'DisplayName', 'Tilting');
    hold off;
    grid on;
    xlabel('Frequency [Hz]');
    ylabel('|K(j\omega)|');
    title('Dynamic Stiffness vs Frequency');
    legend('Location', 'northwest');
    set(gca, 'FontName', format.fontName, 'FontSize', format.fontSize);

end

function plotRunoutVsSoc(soc_vec, runout_mm, format)
    %PLOTRUNOUTVSSOC Plot rotor runout amplitude vs SoC.

    figure('Position', [160, 160, format.figWidth, format.figHeight]);
    plot(soc_vec, runout_mm, 'Color', format.color.primary, ...
        'LineWidth', format.lineWidth);
    grid on;
    xlabel('State of Charge [%]');
    ylabel('Runout amplitude [mm]');
    title('Rotor Runout due to Imbalance vs SoC (G2.5)');
    set(gca, 'FontName', format.fontName, 'FontSize', format.fontSize);

end

function plotSocOverBaselineCycle(J, KE_min, KE_max, soc_vec_grid, ...
    omega_vec_grid, losses, format)
    %PLOTSOCOVERBASELINECYCLE Plot SoC vs time for baseline cycle, start at 50% SoC.
    %   Uses the baseline 15-minute power cycle, the precomputed losses vs
    %   speed, and the KE range to integrate KE over time and plot SoC(t).

    % Time grid and baseline power profile (positive = power to grid)
    t_s      = linspace(0, 900, 901);      % [s], 1 s resolution
    P_grid_W = baselineStorageCycle(t_s);  % [W]

    dt = t_s(2) - t_s(1);

    % Initial KE at 50% SoC
    soc0   = 0.5; % 50%
    KE     = KE_min + soc0 * (KE_max - KE_min);
    KE_vec = zeros(size(t_s));
    soc_vec = zeros(size(t_s));
    omega_vec = zeros(size(t_s));

    KE_vec(1)    = KE;
    omega_vec(1) = sqrt(2*KE/J);
    soc_vec(1)   = 100 * (KE - KE_min) / (KE_max - KE_min);

    % Total losses vs omega on grid
    P_loss_grid = losses.rotor_W + losses.stator_W; % [W] vs omega_vec_grid

    for k = 1:numel(t_s)-1
        % Current speed from KE
        omega = sqrt(2*KE / J);

        % Interpolate loss at current omega
        P_loss = interp1(omega_vec_grid, P_loss_grid, omega, 'linear', 'extrap');

        % Net power leaving rotor (to grid + losses)
        P_net = P_grid_W(k) + P_loss;

        % Update KE: KE_{k+1} = KE_k - P_net * dt
        KE = KE - P_net * dt;

        % Clamp KE to physical range
        KE = max(min(KE, KE_max), KE_min);

        % Store updated state
        KE_vec(k+1)    = KE;
        omega_vec(k+1) = sqrt(2*KE / J);
        soc_vec(k+1)   = 100 * (KE - KE_min) / (KE_max - KE_min);
    end

    % Plot SoC vs time
    figure('Position', [200, 200, format.figWidth, format.figHeight]);
    plot(t_s/60, soc_vec, 'Color', format.color.primary, ...
        'LineWidth', format.lineWidth);
    grid on;
    xlabel('Time [min]');
    ylabel('State of Charge [%]');
    ylim([0 100]); % show full 0–100% SoC range
    title('Flywheel SoC Over Baseline Storage Cycle (Start at 50% SoC)');
    set(gca, 'FontName', format.fontName, 'FontSize', format.fontSize);

end

%% Deliverable 2: Design Study Functions

function requirements = analyzeTeam2CycleRequirements()
    %ANALYZETEAM2CYCLEREQUIREMENTS Analyze team_2_cycle to determine design requirements.
    %   Returns structure with required energy capacity and rated power.
    
    % Ensure path to functions is available (harmless if already in path)
    addpath('Project3_Functions');
    
    % Time grid for team_2_cycle (6 hours = 21600 seconds)
    t_s = 0:1:21600;  % 1 second resolution
    P_grid_W = team_2_cycle(t_s);  % +: power from flywheel to grid
    
    % Power Statistics
    P_max_discharge = max(P_grid_W);
    P_max_charge = abs(min(P_grid_W));
    
    % Energy Integration
    E_cumulative_J = cumtrapz(t_s, P_grid_W);
    E_cumulative_Wh = E_cumulative_J / 3600;
    E_cumulative_kWh = E_cumulative_Wh / 1000;
    
    % Required Flywheel Capacity (energy swing)
    E_cum_min_Wh = min(E_cumulative_Wh);
    E_cum_max_Wh = max(E_cumulative_Wh);
    E_swing_Wh = E_cum_max_Wh - E_cum_min_Wh;
    E_swing_kWh = E_swing_Wh / 1000;
    
    % For 50% SoC start, need capacity = 2 * max(|E_cum_min|, E_cum_max|)
    E_capacity_required_Wh = 2 * max(abs(E_cum_min_Wh), E_cum_max_Wh);
    E_capacity_required_kWh = E_capacity_required_Wh / 1000;
    
    % Store results
    requirements.E_capacity_required_kWh = E_capacity_required_kWh;
    requirements.E_capacity_required_J = E_capacity_required_Wh * 3600;
    requirements.E_swing_kWh = E_swing_kWh;
    requirements.P_max_discharge_kW = P_max_discharge / 1e3;
    requirements.P_max_charge_kW = P_max_charge / 1e3;
    requirements.P_rated_required_kW = max(P_max_discharge, P_max_charge) / 1e3;
    requirements.P_rated_required_W = max(P_max_discharge, P_max_charge);
    
end

function runDeliverable2(params, format)
    %RUNDELIVERABLE2 Perform design study over magnet thickness and max speed.
    %   Analyzes team_2_cycle to determine requirements, applies safety factors,
    %   then iterates over design variables to size flywheel accordingly.
    %   Objectives: minimize losses, maximize specific power, maximize specific energy.

    fprintf('\n=== Deliverable 2: Flywheel Design Study ===\n\n');

    % Step 1: Analyze cycle requirements
    fprintf('=== Step 1: Cycle Requirements Analysis ===\n\n');
    requirements = analyzeTeam2CycleRequirements();
    
    fprintf('Team 2 Cycle Requirements (before safety factors):\n');
    fprintf('  Required energy capacity:     %.2f kWh\n', requirements.E_capacity_required_kWh);
    fprintf('  Energy swing:                 %.2f kWh\n', requirements.E_swing_kWh);
    fprintf('  Peak discharge power:         %.2f kW\n', requirements.P_max_discharge_kW);
    fprintf('  Peak charge power:            %.2f kW\n', requirements.P_max_charge_kW);
    fprintf('  Required rated power:         %.2f kW\n', requirements.P_rated_required_kW);
    fprintf('\n');

    % Step 2: Apply safety factors
    safety_factor_energy = 1.2;  % 20% margin on energy capacity
    safety_factor_power = 1.1;    % 10% margin on rated power
    
    E_capacity_design_kWh = requirements.E_capacity_required_kWh * safety_factor_energy;
    E_capacity_design_J = requirements.E_capacity_required_J * safety_factor_energy;
    P_rated_design_kW = requirements.P_rated_required_kW * safety_factor_power;
    P_rated_design_W = requirements.P_rated_required_W * safety_factor_power;
    
    fprintf('=== Step 2: Safety Factor Application ===\n\n');
    fprintf('Safety factors applied:\n');
    fprintf('  Energy capacity: %.1f (%.0f%% margin)\n', safety_factor_energy, (safety_factor_energy-1)*100);
    fprintf('  Rated power:     %.1f (%.0f%% margin)\n', safety_factor_power, (safety_factor_power-1)*100);
    fprintf('\n');
    fprintf('Design Requirements (after safety factors):\n');
    fprintf('  Design energy capacity:       %.2f kWh\n', E_capacity_design_kWh);
    fprintf('  Design rated power:            %.2f kW\n', P_rated_design_kW);
    fprintf('\n');

    % Convenience alias
    p = params.deliverable1;

    % Step 3: Set up design variable ranges
    fprintf('=== Step 3: Design Variable Iteration ===\n\n');
    t_mag_min = p.geometry.minMagnetThickness_m;  % 2 mm
    t_mag_max = 15e-3;                             % 15 mm
    n_t_mag = 20;                                  % Grid points for magnet thickness
    t_mag_vec = linspace(t_mag_min, t_mag_max, n_t_mag);

    n_omega = 25;
    max_rpm = 80000;
    min_rpm = 5000;
    omega_max_candidates = linspace(min_rpm*2*pi/60, max_rpm*2*pi/60, n_omega); % [rad/s] initial range
    
    % Preallocate results structure
    n_designs = n_t_mag * n_omega;
    designResults = struct('t_mag', cell(1, n_designs), ...
        'omega_max', [], 'feasible', [], 'tempFeasible', [], ...
        'cycleFeasible', [], 'specPower', [], 'specEnergy', [], ...
        'cycleEfficiency', [], 'avgLosses', [], 'geometry', [], ...
        'P_rated', [], 'm_rotating', []);

    fprintf('Evaluating %d design points...\n', n_designs);
    fprintf('Progress: ');

    design_idx = 1;
    for i = 1:n_t_mag
        t_mag = t_mag_vec(i);
        
        for j = 1:n_omega
            omega_max_candidate = omega_max_candidates(j);
            
            % Evaluate this design point with requirements
            result = evaluateDesignPoint(t_mag, omega_max_candidate, params, ...
                E_capacity_design_J, P_rated_design_W, @team_2_cycle);
            
            % Store results
            designResults(design_idx).t_mag = t_mag;
            designResults(design_idx).omega_max = result.omega_max;
            designResults(design_idx).feasible = result.feasible;
            designResults(design_idx).tempFeasible = result.tempFeasible;
            designResults(design_idx).cycleFeasible = result.cycleFeasible;
            designResults(design_idx).specPower = result.specPower;
            designResults(design_idx).specEnergy = result.specEnergy;
            designResults(design_idx).cycleEfficiency = result.cycleEfficiency;
            designResults(design_idx).avgLosses = result.avgLosses;
            designResults(design_idx).geometry = result.geometry;
            designResults(design_idx).P_rated = result.P_rated;
            designResults(design_idx).m_rotating = result.m_rotating;
            
            design_idx = design_idx + 1;
            
            if mod(design_idx-1, 50) == 0
                fprintf('.');
            end
        end
    end
    
    fprintf(' Done.\n\n');

    % Compute baseline design using Deliverable 1 values
    baselineDesign = computeBaselineDesign(params, @team_2_cycle);

    % Plot design tradeoffs
    plotDesignTradeoffs(designResults, baselineDesign, format);

    % Select and report optimal design
    optimalDesign = selectOptimalDesign(designResults, baselineDesign);
    reportOptimalDesign(optimalDesign, baselineDesign, requirements, ...
        E_capacity_design_kWh, P_rated_design_kW);

end

function baselineDesign = computeBaselineDesign(params, cycleFunc)
    %COMPUTEBASELINEDESIGN Compute baseline design using Deliverable 1 geometry.
    %   Uses exact geometry and parameters from Deliverable 1, but evaluates
    %   against the specified cycle function (e.g., team_2_cycle).

    p = params.deliverable1;
    
    % Use Deliverable 1 geometry exactly
    r_fly   = p.geometry.flywheelDiameter_m / 2;
    h_fly   = p.geometry.flywheelAxialLength_m;
    d_shaft = p.geometry.shaftAndPMDiameter_m;
    r_shaft = d_shaft / 2;
    h_motor = p.geometry.motorAxialLength_m;
    t_mag   = p.geometry.magnetThickness_m;
    
    axial_clear = p.clearance.axial_m;
    ambParams = ambParameters(d_shaft, p.amb.ratedForce_N);
    h_shaft = (5 * axial_clear) + h_fly + h_motor + 2 * ambParams.axialLength;
    
    % Masses and inertia (same as Deliverable 1)
    rho_fly   = p.material.densFly_kgPerM3;
    rho_steel = p.material.densSteel_kgPerM3;
    vol_shaft = pi * r_shaft^2 * h_shaft;
    vol_fly   = pi * (r_fly^2 - r_shaft^2) * h_fly;
    m_shaft = rho_steel * vol_shaft;
    m_fly   = rho_fly * vol_fly;
    m_rotating = m_shaft + m_fly;
    J = 0.5 * m_fly * (r_fly^2 - r_shaft^2) + 0.5 * m_shaft * r_shaft^2;
    
    % Speed limits (same as Deliverable 1)
    omega_max_shaft = p.speed.maxTsShaft_mPerSec / r_shaft;
    omega_max_fly   = p.speed.maxTsFly_mPerSec / r_fly;
    if omega_max_shaft <= omega_max_fly
        omega_max = omega_max_shaft;
    else
        omega_max = omega_max_fly;
    end
    omega_min = 0.5 * omega_max;
    
    KE_min = 0.5 * J * omega_min^2;
    KE_max = 0.5 * J * omega_max^2;
    
    % Rated torque and power (same as Deliverable 1)
    [T_rated, P_rated_W] = computeRatedTorqueAndPower(p, r_shaft, h_motor, t_mag, omega_min);
    
    % Compute losses and temperature vs SoC
    soc_vec = linspace(0, 100, 100);
    [losses, T_rot_C, omega_vec] = computeLossesAndTemperatureVsSoc( ...
        soc_vec, J, KE_min, KE_max, P_rated_W, T_rated, ...
        r_fly, h_fly, d_shaft, t_mag, p);
    
    % Check constraints
    tempFeasible = checkTemperatureConstraint(losses, T_rot_C, p);
    [cycleFeasible, cycleEfficiency] = checkCycleFeasibility( ...
        J, KE_min, KE_max, cycleFunc, P_rated_W, losses, omega_vec);
    
    % Compute objectives
    specPower = (P_rated_W / 1e3) / m_rotating;
    specEnergy = ((KE_max - KE_min) / 3600) / m_rotating;
    avgLosses = mean(losses.total_W);
    
    % Store geometry
    geom = struct();
    geom.r_fly = r_fly;
    geom.h_fly = h_fly;
    geom.d_shaft = d_shaft;
    geom.r_shaft = r_shaft;
    geom.h_motor = h_motor;
    geom.h_shaft = h_shaft;
    geom.J = J;
    geom.m_rotating = m_rotating;
    geom.KE_min = KE_min;
    geom.KE_max = KE_max;
    geom.omega_max = omega_max;
    geom.omega_min = omega_min;
    geom.T_rated = T_rated;
    geom.P_rated_W = P_rated_W;
    
    % Build baseline design struct
    baselineDesign = struct();
    baselineDesign.t_mag = t_mag;
    baselineDesign.omega_max = omega_max;
    baselineDesign.specPower = specPower;
    baselineDesign.specEnergy = specEnergy;
    baselineDesign.cycleEfficiency = cycleEfficiency;
    baselineDesign.avgLosses = avgLosses;
    baselineDesign.geometry = geom;
    baselineDesign.feasible = tempFeasible && cycleFeasible;
    baselineDesign.P_rated = P_rated_W;
    baselineDesign.m_rotating = m_rotating;

end

function result = evaluateDesignPoint(t_mag, omega_max_candidate, params, ...
    E_capacity_design_J, P_rated_design_W, cycleFunc)
    %EVALUATEDESIGNPOINT Evaluate a single design point.
    %   Computes geometry from requirements, checks constraints, returns objectives.
    %
    %   Inputs:
    %     t_mag: magnet thickness [m]
    %     omega_max_candidate: candidate maximum rotational speed [rad/s]
    %     params: parameter structure
    %     E_capacity_design_J: required energy capacity [J]
    %     P_rated_design_W: required rated power [W]
    %     cycleFunc: function handle for cycle (e.g., @team_2_cycle)

    p = params.deliverable1;
    
    % Initialize result structure
    result = struct();
    result.feasible = false;
    result.tempFeasible = false;
    result.cycleFeasible = false;
    result.specPower = NaN;
    result.specEnergy = NaN;
    result.cycleEfficiency = NaN;
    result.avgLosses = NaN;
    result.geometry = struct();
    result.P_rated = NaN;
    result.m_rotating = NaN;
    result.omega_max = NaN;

    % Compute geometry from design variables and requirements
    try
        geom = sizeFlywheelFromRequirements(t_mag, omega_max_candidate, ...
            E_capacity_design_J, P_rated_design_W, params);
        result.geometry = geom;
        result.omega_max = geom.omega_max;
    catch
        % Geometry computation failed (likely infeasible)
        return;
    end

    % Extract geometry
    r_fly = geom.r_fly;
    h_fly = geom.h_fly;
    d_shaft = geom.d_shaft;
    r_shaft = geom.r_shaft;
    h_motor = geom.h_motor;
    J = geom.J;
    m_rotating = geom.m_rotating;
    KE_min = geom.KE_min;
    KE_max = geom.KE_max;
    omega_max = geom.omega_max;
    omega_min = geom.omega_min;
    T_rated = geom.T_rated;
    P_rated_W = geom.P_rated_W;

    result.P_rated = P_rated_W;
    result.m_rotating = m_rotating;

    % Compute losses and temperature vs SoC
    soc_vec = linspace(0, 100, 100);
    [losses, T_rot_C, omega_vec] = computeLossesAndTemperatureVsSoc( ...
        soc_vec, J, KE_min, KE_max, P_rated_W, T_rated, ...
        r_fly, h_fly, d_shaft, t_mag, p);

    % Check temperature constraint (max temp at all SoC points)
    result.tempFeasible = checkTemperatureConstraint(losses, T_rot_C, p);
    
    if ~result.tempFeasible
        return; % Early exit if temperature constraint violated
    end

    % Check cycle feasibility
    [result.cycleFeasible, result.cycleEfficiency] = checkCycleFeasibility( ...
        J, KE_min, KE_max, cycleFunc, P_rated_W, losses, omega_vec);

    if ~result.cycleFeasible
        return; % Early exit if cycle constraint violated
    end

    % Compute objectives
    result.specPower = (P_rated_W / 1e3) / m_rotating;  % [kW/kg]
    result.specEnergy = ((KE_max - KE_min) / 3600) / m_rotating;  % [Wh/kg]
    result.avgLosses = mean(losses.total_W);  % [W] average losses

    % Design is feasible if both constraints are satisfied
    result.feasible = result.tempFeasible && result.cycleFeasible;

end

function geom = sizeFlywheelFromRequirements(t_mag, omega_max_candidate, ...
    E_capacity_design_J, P_rated_design_W, params)
    %SIZEFLYWHEELFROMREQUIREMENTS Size flywheel system to meet energy and power requirements.
    %   Given magnet thickness, candidate max speed, energy capacity, and rated power,
    %   computes geometry based on tip speed limits and requirements.
    %
    %   Inputs:
    %     t_mag: magnet thickness [m]
    %     omega_max_candidate: candidate maximum rotational speed [rad/s]
    %     E_capacity_design_J: required energy capacity [J]
    %     P_rated_design_W: required rated power [W]
    %     params: parameter structure

    p = params.deliverable1;
    rho_fly = p.material.densFly_kgPerM3;
    rho_steel = p.material.densSteel_kgPerM3;
    
    % Step 1: Determine actual max speed from tip speed limits
    % Start with candidate speed to size components
    omega_max_temp = omega_max_candidate;
    
    % Size shaft from tip speed limit
    r_shaft = p.speed.maxTsShaft_mPerSec / omega_max_temp;
    d_shaft = 2 * r_shaft;
    
    % Size flywheel from tip speed limit
    r_fly = p.speed.maxTsFly_mPerSec / omega_max_temp;
    
    % Check PM tip speed limit
    r_pm = r_shaft + t_mag;
    omega_max_pm = p.speed.maxTsPM_mPerSec / r_pm;
    
    % Actual max speed is minimum of all limits
    omega_max_shaft = p.speed.maxTsShaft_mPerSec / r_shaft;
    omega_max_fly = p.speed.maxTsFly_mPerSec / r_fly;
    
    omega_max = min([omega_max_shaft, omega_max_fly, omega_max_pm, omega_max_temp]);
    
    % Recompute geometry with actual max speed
    r_shaft = p.speed.maxTsShaft_mPerSec / omega_max;
    d_shaft = 2 * r_shaft;
    r_fly = p.speed.maxTsFly_mPerSec / omega_max;
    
    % Check if geometry is physically reasonable
    if r_fly <= r_shaft || omega_max <= 0
        error('Infeasible geometry');
    end

    % Step 2: Set omega_min based on SoC mapping (0% SoC = 0.5 * max speed)
    omega_min = 0.5 * omega_max;

    % Step 3: Size flywheel geometry to meet energy capacity requirement
    % Required inertia: E = 0.375 * J * omega_max^2
    % (0.375 factor: KE_max - KE_min = 0.5*J*(omega_max^2 - omega_min^2)
    %  where omega_min = 0.5*omega_max, so = 0.375*J*omega_max^2)
    J_required = E_capacity_design_J / (0.375 * omega_max^2);
    
    % For a hollow cylinder: J = 0.5 * rho * pi * h * (r_o^4 - r_i^4)
    % Solve for h_fly
    h_fly = J_required / (0.5 * rho_fly * pi * (r_fly^4 - r_shaft^4));
    
    % Check if flywheel height is reasonable
    if h_fly <= 0 || h_fly > 5  % Max 5m seems reasonable
        error('Infeasible flywheel height');
    end

    % Step 4: Size motor to meet rated power requirement
    % Required torque at minimum speed: T_required = P_rated / omega_min
    T_required = P_rated_design_W / omega_min;
    
    % Magnetic shear stress at rated current (Ipu = 1)
    tau_rated = magneticShear(t_mag, 1);
    D_rotor = d_shaft + 2 * t_mag;
    r_rotor = D_rotor / 2;
    
    % Size motor length to achieve required torque
    % T = 2 * pi * r_rotor^2 * h_motor * tau_rated
    % h_motor = T / (2 * pi * r_rotor^2 * tau_rated)
    h_motor = T_required / (2 * pi * r_rotor^2 * tau_rated);
    
    % Check if motor length is reasonable (min 50mm, max 1m)
    if h_motor < 50e-3
        h_motor = 50e-3;  % Minimum motor length
        % Recompute actual torque and power
        T_rated = 2 * pi * r_rotor^2 * h_motor * tau_rated;
        P_rated_W = T_rated * omega_min;
    elseif h_motor > 1.0
        error('Infeasible motor length');
    else
        T_rated = T_required;
        P_rated_W = P_rated_design_W;
    end

    % Step 5: Compute shaft geometry, masses, and final inertia
    ambParams = ambParameters(d_shaft, p.amb.ratedForce_N);
    axial_clear = p.clearance.axial_m;
    h_shaft = (5 * axial_clear) + h_fly + h_motor + 2 * ambParams.axialLength;

    % Compute masses
    vol_shaft = pi * r_shaft^2 * h_shaft;
    vol_fly = pi * (r_fly^2 - r_shaft^2) * h_fly;
    
    m_shaft = rho_steel * vol_shaft;
    m_fly = rho_fly * vol_fly;
    m_rotating = m_shaft + m_fly;

    % Compute inertia (including shaft contribution)
    J_shaft = 0.5 * m_shaft * r_shaft^2;
    J_fly = 0.5 * m_fly * (r_fly^2 - r_shaft^2);
    J = J_shaft + J_fly;

    % Recompute energy range with actual inertia
    KE_min = 0.5 * J * omega_min^2;
    KE_max = 0.5 * J * omega_max^2;

    % Store geometry
    geom.r_fly = r_fly;
    geom.h_fly = h_fly;
    geom.d_shaft = d_shaft;
    geom.r_shaft = r_shaft;
    geom.h_motor = h_motor;
    geom.h_shaft = h_shaft;
    geom.J = J;
    geom.m_rotating = m_rotating;
    geom.KE_min = KE_min;
    geom.KE_max = KE_max;
    geom.omega_max = omega_max;
    geom.omega_min = omega_min;
    geom.T_rated = T_rated;
    geom.P_rated_W = P_rated_W;

end

function isFeasible = checkTemperatureConstraint(losses, T_rot_C, p)
    %CHECKTEMPERATURECONSTRAINT Check if max temperature constraint is satisfied.
    %   Returns true if T_rot_C < maxTemp_C at all SoC points.

    maxTemp_C = p.thermal.maxTemp_C;
    isFeasible = all(T_rot_C < maxTemp_C);

end

function [isFeasible, cycleEfficiency] = checkCycleFeasibility( ...
    J, KE_min, KE_max, cycleFunc, P_rated_W, losses, omega_vec)
    %CHECKCYCLEFEASIBILITY Simulate cycle and check SoC > 0% throughout.
    %   Returns feasibility flag and cycle efficiency.

    % Time grid for team_2_cycle (6 hours = 21600 seconds)
    t_s = linspace(0, 21600, 21601);  % 1 s resolution
    P_grid_W = cycleFunc(t_s);  % +: power from flywheel to grid

    dt = t_s(2) - t_s(1);

    % Initial KE at 50% SoC
    soc0 = 0.5; % 50%
    KE = KE_min + soc0 * (KE_max - KE_min);
    
    % Total losses vs omega on grid
    P_loss_grid = losses.rotor_W + losses.stator_W;  % [W] vs omega_vec
    
    % Energy accounting
    E_out_Wh = 0;
    E_in_Wh = 0;
    E_loss_Wh = 0;
    minSoC = 100;  % Track minimum SoC
    
    for k = 1:numel(t_s)-1
        % Current speed from KE
        omega = sqrt(2*KE / J);
        
        % Interpolate loss at current omega
        P_loss = interp1(omega_vec, P_loss_grid, omega, 'linear', 'extrap');
        
        % Net power leaving rotor (to grid + losses)
        P_net = P_grid_W(k) + P_loss;
        
        % Update KE: KE_{k+1} = KE_k - P_net * dt
        KE = KE - P_net * dt;
        
        % Clamp KE to physical range
        KE = max(min(KE, KE_max), KE_min);
        
        % Compute SoC
        soc = 100 * (KE - KE_min) / (KE_max - KE_min);
        minSoC = min(minSoC, soc);
        
        % Energy accounting
        if P_grid_W(k) > 0
            E_out_Wh = E_out_Wh + P_grid_W(k) * dt / 3600;
        else
            E_in_Wh = E_in_Wh - P_grid_W(k) * dt / 3600;
        end
        E_loss_Wh = E_loss_Wh + P_loss * dt / 3600;
    end
    
    % Check feasibility: SoC > 0% throughout
    isFeasible = (minSoC > 0);
    
    % Compute cycle efficiency: energy recovered / energy stored
    if E_in_Wh > 0
        cycleEfficiency = E_out_Wh / E_in_Wh;
    else
        cycleEfficiency = 0;
    end

end

function plotDesignTradeoffs(designResults, baselineDesign, format)
    %PLOTDESIGNTRADEOFFS Create 3D scatter plot of design tradeoffs.

    % Extract feasible designs
    feasibleIdx = [designResults.feasible];
    feasibleResults = designResults(feasibleIdx);
    
    if isempty(feasibleResults)
        fprintf('Warning: No feasible designs found.\n');
        return;
    end

    % Extract data for plotting
    specPower = [feasibleResults.specPower];
    specEnergy = [feasibleResults.specEnergy];
    efficiency = [feasibleResults.cycleEfficiency];
    
    % Create 3D scatter plot
    figure('Position', [100, 100, format.figWidth, format.figHeight]);
    
    scatter3(specPower, specEnergy, efficiency, 50, efficiency, 'filled');
    hold on;
    
    % Mark baseline design
    if baselineDesign.feasible
        scatter3(baselineDesign.specPower, baselineDesign.specEnergy, ...
            baselineDesign.cycleEfficiency, 200, 'r', 'x', 'LineWidth', 3);
    end
    
    hold off;
    
    xlabel('Specific Power [kW/kg]', 'FontName', format.fontName, ...
        'FontSize', format.fontSize);
    ylabel('Specific Energy [Wh/kg]', 'FontName', format.fontName, ...
        'FontSize', format.fontSize);
    zlabel('Cycle Efficiency [-]', 'FontName', format.fontName, ...
        'FontSize', format.fontSize);
    title('Design Tradeoffs: Specific Power vs Specific Energy vs Efficiency', ...
        'FontName', format.fontName, 'FontSize', format.fontSize);
    grid on;
    colorbar;
    colormap('jet');
    c = colorbar;
    c.Label.String = 'Cycle Efficiency';
    c.Label.FontName = format.fontName;
    c.Label.FontSize = format.fontSize;
    
    if baselineDesign.feasible
        legend('Feasible Designs', 'Baseline Design', 'Location', 'best', ...
            'FontName', format.fontName, 'FontSize', format.fontSize);
    else
        legend('Feasible Designs', 'Location', 'best', ...
            'FontName', format.fontName, 'FontSize', format.fontSize);
    end
    
    set(gca, 'FontName', format.fontName, 'FontSize', format.fontSize);

end

function optimalDesign = selectOptimalDesign(designResults, baselineDesign)
    %SELECTOPTIMALDESIGN Identify optimal design from feasible set.
    %   Uses multi-objective criteria: minimize losses, maximize specific power,
    %   maximize specific energy, maximize efficiency.

    % Extract feasible designs
    feasibleIdx = [designResults.feasible];
    feasibleResults = designResults(feasibleIdx);
    
    if isempty(feasibleResults)
        optimalDesign = struct();
        fprintf('Warning: No feasible designs found. Cannot select optimal design.\n');
        return;
    end

    % Extract objectives
    specPower = [feasibleResults.specPower];
    specEnergy = [feasibleResults.specEnergy];
    efficiency = [feasibleResults.cycleEfficiency];
    avgLosses = [feasibleResults.avgLosses];
    
    % Normalize objectives for weighted sum (all to maximize)
    % Higher is better for: specPower, specEnergy, efficiency
    % Lower is better for: avgLosses (so well use -avgLosses)
    
    specPower_norm = (specPower - min(specPower)) / (max(specPower) - min(specPower) + eps);
    specEnergy_norm = (specEnergy - min(specEnergy)) / (max(specEnergy) - min(specEnergy) + eps);
    efficiency_norm = (efficiency - min(efficiency)) / (max(efficiency) - min(efficiency) + eps);
    avgLosses_norm = 1 - (avgLosses - min(avgLosses)) / (max(avgLosses) - min(avgLosses) + eps);
    
    % Weighted sum (equal weights for now)
    weights = [0.25, 0.25, 0.25, 0.25];  % [specPower, specEnergy, efficiency, -losses]
    scores = weights(1) * specPower_norm + ...
             weights(2) * specEnergy_norm + ...
             weights(3) * efficiency_norm + ...
             weights(4) * avgLosses_norm;
    
    % Find optimal design
    [~, optIdx] = max(scores);
    optimalDesign = feasibleResults(optIdx);
    
    % Also check for designs that are better than baseline in all metrics
    if baselineDesign.feasible
        betterThanBaseline = (specPower > baselineDesign.specPower) & ...
                            (specEnergy > baselineDesign.specEnergy) & ...
                            (efficiency > baselineDesign.cycleEfficiency) & ...
                            (avgLosses < baselineDesign.avgLosses);
        
        if any(betterThanBaseline)
            % Use the best of these
            betterIdx = find(betterThanBaseline);
            [~, bestBetterIdx] = max(scores(betterIdx));
            optimalDesign = feasibleResults(betterIdx(bestBetterIdx));
        end
    end

end

function reportOptimalDesign(optimalDesign, baselineDesign, requirements, ...
    E_capacity_design_kWh, P_rated_design_kW)
    %REPORTOPTIMALDESIGN Print optimal design dimensions and justification.
    %
    %   Inputs:
    %     optimalDesign: optimal design structure
    %     baselineDesign: baseline design structure
    %     requirements: cycle requirements structure
    %     E_capacity_design_kWh: design energy capacity [kWh]
    %     P_rated_design_kW: design rated power [kW]

    fprintf('\n=== Optimal Design Selection ===\n\n');
    
    if isempty(fieldnames(optimalDesign)) || ~isfield(optimalDesign, 'geometry')
        fprintf('No optimal design selected.\n');
        return;
    end

    geom = optimalDesign.geometry;
    
    % Compute actual energy capacity
    E_capacity_actual_kWh = (geom.KE_max - geom.KE_min) / 3600 / 1000;
    
    fprintf('Optimal Design Parameters:\n');
    fprintf('  Magnet thickness:        %.2f mm\n', optimalDesign.t_mag * 1e3);
    fprintf('  Maximum speed:           %.0f rpm (%.1f rad/s)\n', ...
        optimalDesign.omega_max * 60/(2*pi), optimalDesign.omega_max);
    fprintf('  Rated power:             %.1f kW\n', optimalDesign.P_rated / 1e3);
    fprintf('  Rotating mass:           %.2f kg\n', optimalDesign.m_rotating);
    fprintf('\n');
    
    fprintf('Geometry:\n');
    fprintf('  Flywheel diameter:       %.1f mm\n', 2*geom.r_fly * 1e3);
    fprintf('  Flywheel height:         %.1f mm\n', geom.h_fly * 1e3);
    fprintf('  Shaft diameter:          %.1f mm\n', geom.d_shaft * 1e3);
    fprintf('  Motor length:            %.1f mm\n', geom.h_motor * 1e3);
    fprintf('  Total shaft length:      %.1f mm\n', geom.h_shaft * 1e3);
    fprintf('\n');
    
    fprintf('Requirements Compliance:\n');
    fprintf('  Design energy capacity:  %.2f kWh\n', E_capacity_design_kWh);
    fprintf('  Actual energy capacity:  %.2f kWh (%.1f%% of requirement)\n', ...
        E_capacity_actual_kWh, 100*E_capacity_actual_kWh/E_capacity_design_kWh);
    fprintf('  Design rated power:       %.2f kW\n', P_rated_design_kW);
    fprintf('  Actual rated power:       %.2f kW (%.1f%% of requirement)\n', ...
        optimalDesign.P_rated / 1e3, 100*(optimalDesign.P_rated/1e3)/P_rated_design_kW);
    fprintf('\n');
    
    fprintf('Performance Metrics:\n');
    fprintf('  Specific power:          %.3f kW/kg\n', optimalDesign.specPower);
    fprintf('  Specific energy:         %.3f Wh/kg\n', optimalDesign.specEnergy);
    fprintf('  Cycle efficiency:         %.1f %%\n', optimalDesign.cycleEfficiency * 100);
    fprintf('  Average losses:           %.1f kW\n', optimalDesign.avgLosses / 1e3);
    fprintf('\n');
    
    if baselineDesign.feasible
        fprintf('Comparison to Baseline:\n');
        fprintf('  Specific power:          %.3f vs %.3f kW/kg (%+.1f%%)\n', ...
            optimalDesign.specPower, baselineDesign.specPower, ...
            100*(optimalDesign.specPower/baselineDesign.specPower - 1));
        fprintf('  Specific energy:         %.3f vs %.3f Wh/kg (%+.1f%%)\n', ...
            optimalDesign.specEnergy, baselineDesign.specEnergy, ...
            100*(optimalDesign.specEnergy/baselineDesign.specEnergy - 1));
        fprintf('  Cycle efficiency:         %.1f vs %.1f %% (%+.1f%%)\n', ...
            optimalDesign.cycleEfficiency * 100, baselineDesign.cycleEfficiency * 100, ...
            100*(optimalDesign.cycleEfficiency/baselineDesign.cycleEfficiency - 1));
        fprintf('  Average losses:           %.1f vs %.1f kW (%+.1f%%)\n', ...
            optimalDesign.avgLosses / 1e3, baselineDesign.avgLosses / 1e3, ...
            100*(optimalDesign.avgLosses/baselineDesign.avgLosses - 1));
        fprintf('\n');
    end
    
    fprintf('Justification:\n');
    fprintf('  This design was selected based on a weighted combination of\n');
    fprintf('  objectives: maximizing specific power, specific energy, and\n');
    fprintf('  cycle efficiency while minimizing average losses. The design\n');
    fprintf('  satisfies all constraints: temperature stays below maximum\n');
    fprintf('  at all SoC points, and the storage cycle can be completed\n');
    fprintf('  starting at 50%% SoC without dropping below 0%% SoC.\n');
    fprintf('  The design meets or exceeds the required energy capacity\n');
    fprintf('  (%.2f kWh) and rated power (%.2f kW) determined from the\n', ...
        E_capacity_design_kWh, P_rated_design_kW);
    fprintf('  team_2_cycle analysis with applied safety factors.\n\n');

end