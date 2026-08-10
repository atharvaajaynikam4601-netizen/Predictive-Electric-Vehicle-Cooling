classdef PredictiveLookaheadController < matlab.System
    % PredictiveLookaheadController  Forecast-aware pre-cooling controller
    % for the Simscape Fluids EV_Predictive_Cooling_Plant_PredictiveLookahead
    % model.
    %
    % Reuses the heat-generation-lookahead and proportional setpoint logic
    % from Controller 2 in scripts/Dynamic_Loads.m (Section 4.8 of the
    % project README): it forecasts the mean battery heat generation over
    % the next Hp seconds and, once the battery is at or above the target
    % setpoint, commands cooling proportional to that forecast plus the
    % current temperature error.
    %
    % SCOPE NOTE: the charger-proximity and throttle-lookahead pre-cool
    % branches from Dynamic_Loads.m are intentionally NOT ported here.
    % This plant is driven by the UDDS-derived Q_heat_ts signal only --
    % there is no route-position or throttle signal wired into it -- so
    % only the heat-generation-forecast branch is meaningful in this
    % context. This is a deliberate, documented scoping decision, not an
    % oversight.
    %
    % Drop-in replacement for the reactive controller's "MATLAB Function"
    % block: single input T_batt_K (K), single output mdot (kg/s).

    properties (Nontunable)
        Hp = 120;               % Lookahead horizon (s)
        T_set = 305.15;         % Target setpoint (K) = 32 degC, matches Section 4.6/4.7
        tau_control = 30;       % Proportional control time constant (s)
        C_th = 45000;           % Thermal capacitance (J/K), matches Section 4.6
        Q_cooling_max = 2000;   % Max cooling power (W), matches reactive baseline cap
        mdot_low = 0.005;       % Minimum coolant flow (kg/s), matches reactive controller
        mdot_high = 0.050;      % Maximum coolant flow (kg/s), matches reactive controller
    end

    properties (Access = private)
        Qts   % full Q_heat timeseries, pulled once from the base workspace
    end

    methods (Access = protected)
        function setupImpl(obj)
            % Q_heat_ts must already be in the base workspace -- run
            % scripts/ev_eneergy_model_realistic_predictive_cooling.m (or
            % run_project) before simulating this model.
            obj.Qts = evalin('base', 'Q_heat_ts');
        end

        function mdot = stepImpl(obj, T_batt_K)
            t = getCurrentTime(obj);

            idx = obj.Qts.Time >= t & obj.Qts.Time <= t + obj.Hp;
            if any(idx)
                Q_future = mean(obj.Qts.Data(idx));
            else
                Q_future = obj.Qts.Data(end);
            end

            if T_batt_K >= obj.T_set
                Q_req = Q_future + obj.C_th * (T_batt_K - obj.T_set) / obj.tau_control;
                Q_req = min(obj.Q_cooling_max, max(0, Q_req));
            else
                Q_req = 0;
            end

            frac = Q_req / obj.Q_cooling_max;
            mdot = obj.mdot_low + frac * (obj.mdot_high - obj.mdot_low);
        end

        function s = getSampleTimeImpl(obj)
            s = createSampleTime(obj, 'Type', 'Discrete', 'SampleTime', 1);
        end
    end
end
