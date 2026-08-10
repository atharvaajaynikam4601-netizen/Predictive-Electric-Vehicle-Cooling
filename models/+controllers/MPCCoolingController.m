classdef MPCCoolingController < matlab.System
    % MPCCoolingController  Receding-horizon constrained nonlinear MPC
    % battery cooling controller for the Simscape Fluids
    % EV_Predictive_Cooling_Plant_MPC model.
    %
    % Solves the identical cost function and hard 35 degC nonlinear
    % constraint as Controller 3 in scripts/Dynamic_Loads.m (Section 4.9
    % of the project README), via fmincon over a receding Hp-second
    % horizon, so the MPC controller benchmarked in MATLAB and the one
    % driving the physical Simscape Fluids plant solve the same
    % optimization problem.
    %
    % fmincon is not code-generation compatible, so this block must be
    % simulated in Interpreted Execution (the default for MATLAB System
    % blocks -- do not mark it for code generation). The solve is also
    % throttled to run once every SampleTime seconds (default 5 s) rather
    % than every solver step, since re-solving a 120-step nonlinear
    % program at every minor time step is not tractable; the commanded
    % flow rate is held constant between solves.
    %
    % Drop-in replacement for the reactive controller's "MATLAB Function"
    % block: single input T_batt_K (K), single output mdot (kg/s).

    properties (Nontunable)
        SampleTime = 5;          % MPC solve interval (s)
        Hp = 120;                 % Prediction horizon (s)
        T_set = 305.15;           % Target setpoint (K) = 32 degC
        T_max_limit = 308.15;     % Hard safety constraint (K) = 35 degC
        C_th = 45000;             % Thermal capacitance (J/K), matches Section 4.6
        R_th = 1.5;               % Thermal resistance to ambient (K/W), matches Section 4.6
        Q_cooling_max = 2000;     % Max cooling power (W), matches reactive baseline cap
        w_T = 100.0;              % Thermal error penalty weight
        w_E = 1e-5;               % Energy penalty weight
        w_dU = 1e-3;              % Actuator slew-rate penalty weight
        mdot_low = 0.005;         % Minimum coolant flow (kg/s)
        mdot_high = 0.050;        % Maximum coolant flow (kg/s)
    end

    properties (Access = private)
        Qts             % full Q_heat timeseries, pulled once from the base workspace
        Tambts          % full T_amb timeseries, pulled once from the base workspace
        u_init          % warm-started decision vector for fmincon
        u_prev           % last applied cooling command (W), for slew-rate cost
        Q_cooling_last  % cooling power held between solves (W)
        lastSolveTime   % simulation time of the last MPC solve (s)
        opts            % fmincon options
    end

    methods (Access = protected)
        function setupImpl(obj)
            % Q_heat_ts / T_amb_ts must already be in the base workspace --
            % run scripts/ev_eneergy_model_realistic_predictive_cooling.m
            % (or run_project) before simulating this model.
            obj.Qts = evalin('base', 'Q_heat_ts');
            obj.Tambts = evalin('base', 'T_amb_ts');
            obj.u_init = zeros(obj.Hp, 1);
            obj.u_prev = 0;
            obj.Q_cooling_last = 0;
            obj.lastSolveTime = -inf;
            obj.opts = optimoptions('fmincon', 'Display', 'off', ...
                'Algorithm', 'sqp', 'MaxIterations', 30, ...
                'OptimalityTolerance', 1e-3);
        end

        function mdot = stepImpl(obj, T_batt_K)
            t = getCurrentTime(obj);

            if t - obj.lastSolveTime >= obj.SampleTime - 1e-9
                obj.lastSolveTime = t;

                tGrid = t + (0:obj.Hp-1)';
                Q_dist = interp1(obj.Qts.Time, obj.Qts.Data, tGrid, 'linear', 'extrap');
                T_a_dist = interp1(obj.Tambts.Time, obj.Tambts.Data, tGrid, 'linear', 'extrap');

                costFun = @(u) controllers.internal.mpcCost(u, T_batt_K, Q_dist, T_a_dist, ...
                    obj.u_prev, obj.C_th, obj.R_th, obj.SampleTime, obj.Hp, obj.T_set, ...
                    obj.w_T, obj.w_E, obj.w_dU);
                nonlconFun = @(u) controllers.internal.mpcNonlcon(u, T_batt_K, Q_dist, T_a_dist, ...
                    obj.C_th, obj.R_th, obj.SampleTime, obj.Hp, obj.T_max_limit);

                lb = zeros(obj.Hp, 1);
                ub = obj.Q_cooling_max * ones(obj.Hp, 1);

                u_opt = fmincon(costFun, obj.u_init, [], [], [], [], lb, ub, nonlconFun, obj.opts);

                obj.Q_cooling_last = u_opt(1);
                obj.u_prev = u_opt(1);
                obj.u_init = [u_opt(2:end); u_opt(end)];
            end

            frac = obj.Q_cooling_last / obj.Q_cooling_max;
            mdot = obj.mdot_low + frac * (obj.mdot_high - obj.mdot_low);
        end

        function s = getSampleTimeImpl(obj)
            s = createSampleTime(obj, 'Type', 'Discrete', 'SampleTime', 1);
            % Stepped every 1 s; the fmincon solve itself is throttled to
            % obj.SampleTime internally via lastSolveTime, above.
        end
    end
end
