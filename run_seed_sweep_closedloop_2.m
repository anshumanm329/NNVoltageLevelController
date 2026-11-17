%% run_seed_sweep_closedloop_2.m
%
% Multi-seed closed-loop validation, built on closed_loop_batch_simulation_2.m.
%
% The model runs all four controllers as parallel plant replicas in one
% simulation, so a single sim per (seed, scenario) yields NN, MPC, PID and
% PID+FF together. Only the NN replica depends on the training seed, which
% this script exploits as a self-check: the MPC, PID and PID+FF metrics must
% come out bit-identical for every seed. If they drift, something other than
% the network changed between runs and the sweep is not trustworthy.
%
% Metric definitions are kept identical to closed_loop_batch_simulation_2.m
% (IAE, ISE, steady-state error, overshoot, settling time) so the numbers
% stay comparable with the results you already have. Extra columns are added
% for the disturbance transient; they are only meaningful for scenarios where
% the load step comes after the setpoint step.
%
% ---------------------------------------------------------------------
% Why the reload dance
% ---------------------------------------------------------------------
% nn_controller_3.m holds W1..b6 in PERSISTENT variables, filled once via
% coder.load('nn_weights_for_matlab_3.mat'). coder.load resolves at model
% compile time and the persistents survive across sim() calls, so simply
% overwriting the .mat between runs would keep executing the first seed's
% network. bdclose + clear functions + load_system forces a fresh compile
% and a fresh load.
%
% Note: the plain load('nn_weights_for_matlab_3.mat') at the top of your
% batch script puts the weights in the BASE workspace, where the MATLAB
% Function block never looks. It has no effect on which network runs.
% ---------------------------------------------------------------------

clear; clc;

%% ============================================================
%  CONFIG
%  ============================================================
model_name     = 'Working_with_RLC_3_5';
target_block   = [model_name '/Target'];
iload_block    = [model_name '/Iload'];

load('mpc_with_fixed_gain.mat');            % expert MPC object

WEIGHTS_TARGET = 'nn_weights_for_matlab_3.mat';  % hard-coded in nn_controller_3.m
WEIGHTS_DIR    = 'nn_seed_weights';
SEEDS          = [48 1 2 3 4 5 6 7 8 9];

LOG_TIME_VAR          = 'tout';
LOG_XDATA_VAR         = 'X_data';
XDATA_SETPOINT_COL    = 1;
XDATA_OUTPUT_COL      = 2;
LOG_MPC_OUTPUT_VAR    = 'simout_mpc_output';
LOG_PID_OUTPUT_VAR    = 'simout_pid_output';
LOG_PID_FF_OUTPUT_VAR = 'simout_pid_ff_output';

StopTime         = 0.1;
SETTLE_BAND_FRAC = 0.02;
OUT_CSV          = 'seed_sweep_closedloop_results.csv';

% Time-series export. One CSV per (seed, scenario) holding the time axis,
% the logged setpoint and all four controller responses, so the figures can
% be redrawn from data instead of read off a scope. Set SAVE_TRACES to
% false if you only want the metrics table.
SAVE_TRACES = true;
TRACE_DIR   = 'closed_loop_traces';

%% ============================================================
%  Scenarios
%  ============================================================
% t_step / t_load are explicit here. The first five reproduce your existing
% batch (both steps at t = 0); the last one is the 19 V / 0.3 A case
% described in Sections 2.6 and 3 of the manuscript, with the setpoint step
% at 0.01 s and the load step at 0.04 s, matching how the training data was
% generated.
scenarios = struct('label', {}, 'setpoint', {}, 'iload', {}, ...
                   't_step', {}, 't_load', {});
scenarios(end+1) = struct('label','on-grid_10V_0.2A', 'setpoint',10,  'iload',0.2,  't_step',0,    't_load',0);
scenarios(end+1) = struct('label','on-grid_12V_0.5A', 'setpoint',12,  'iload',0.5,  't_step',0,    't_load',0);
scenarios(end+1) = struct('label','ood_15V_0.3A',     'setpoint',15,  'iload',0.3,  't_step',0,    't_load',0);
scenarios(end+1) = struct('label','ood_22V_0.15A',    'setpoint',22,  'iload',0.15, 't_step',0,    't_load',0);
scenarios(end+1) = struct('label','novel_offgrid_9.4V_0.37A', 'setpoint',9.4, 'iload',0.37, 't_step',0, 't_load',0);
scenarios(end+1) = struct('label','paper_19V_0.3A',   'setpoint',19,  'iload',0.3,  't_step',0.01, 't_load',0.04);

%% ============================================================
if ~isfolder(WEIGHTS_DIR)
    error('Weights folder "%s" not found.', WEIGHTS_DIR);
end
if SAVE_TRACES && ~isfolder(TRACE_DIR)
    mkdir(TRACE_DIR);
end
if isfile(WEIGHTS_TARGET) && ~isfile([WEIGHTS_TARGET '.bak'])
    copyfile(WEIGHTS_TARGET, [WEIGHTS_TARGET '.bak']);
    fprintf('Original weights backed up to %s.bak\n', WEIGHTS_TARGET);
end

results = table();

for k = 1:numel(SEEDS)
    seed = SEEDS(k);
    src  = fullfile(WEIGHTS_DIR, sprintf('nn_weights_seed%d.mat', seed));
    if ~isfile(src)
        warning('Missing %s - skipping seed %d.', src, seed); continue
    end

    fprintf('\n===== seed %d  (%d/%d) =====\n', seed, k, numel(SEEDS));

    % install this seed's network and force a clean recompile
    bdclose('all');
    clear functions %#ok<CLFUNC>
    copyfile(src, WEIGHTS_TARGET, 'f');
    load_system(model_name);

    for s = 1:numel(scenarios)
        sc = scenarios(s);

        set_param(target_block, 'Time', num2str(sc.t_step), 'Before','0', ...
                  'After', num2str(sc.setpoint));
        set_param(iload_block,  'Time', num2str(sc.t_load), 'Before','0', ...
                  'After', num2str(sc.iload));
        set_param(model_name, 'StopTime', num2str(StopTime));

        fprintf('  %-28s setpoint %.2f V, Iload %.2f A\n', sc.label, sc.setpoint, sc.iload);
        simOut = sim(model_name);

        t = simOut.get(LOG_TIME_VAR); t = t(:);
        X = simOut.get(LOG_XDATA_VAR);
        sp_vec = X(:, XDATA_SETPOINT_COL);

        signals = { ...
            'NN',          X(:, XDATA_OUTPUT_COL); ...
            'MPC',         simOut.get(LOG_MPC_OUTPUT_VAR); ...
            'PID',         simOut.get(LOG_PID_OUTPUT_VAR); ...
            'PID_with_FF', simOut.get(LOG_PID_FF_OUTPUT_VAR)};

        for c = 1:size(signals, 1)
            row = compute_metrics_row(signals{c,1}, sc, t, signals{c,2}(:), ...
                                      sp_vec, SETTLE_BAND_FRAC);
            row.Seed = seed;
            results = [results; row]; %#ok<AGROW>
        end

        if SAVE_TRACES
            trace = table(t, sp_vec, signals{1,2}(:), signals{2,2}(:), ...
                          signals{3,2}(:), signals{4,2}(:), ...
                          'VariableNames', {'time_s','setpoint_V','Vout_NN_V', ...
                          'Vout_MPC_V','Vout_PID_V','Vout_PID_FF_V'});
            fname = fullfile(TRACE_DIR, ...
                     sprintf('trace_seed%d_%s.csv', seed, matlab.lang.makeValidName(sc.label)));
            writetable(trace, fname);
        end
    end
end

results = movevars(results, 'Seed', 'Before', 1);
writetable(results, OUT_CSV);
fprintf('\nWrote %s (%d rows)\n', OUT_CSV, height(results));

%% ---- self-check: the three seed-independent controllers must not move ----
fprintf('\n--- consistency check (MPC / PID / PID+FF must be identical across seeds) ---\n');
metric_cols = {'IAE','ISE','SteadyStateError_V','Overshoot_pct','SettlingTime_s'};
bad = false;
for ctrl = {'MPC','PID','PID_with_FF'}
    sub = results(strcmp(results.Controller, ctrl{1}), :);
    for sname = unique(sub.Scenario)'
        rows = sub(strcmp(sub.Scenario, sname{1}), :);
        for mc = metric_cols
            spread = max(rows.(mc{1})) - min(rows.(mc{1}));
            if spread > 1e-9
                fprintf('  MISMATCH %s / %s / %s: spread %.3e\n', ...
                        ctrl{1}, sname{1}, mc{1}, spread);
                bad = true;
            end
        end
    end
end
if ~bad
    fprintf('  OK - the non-NN controllers are identical for every seed.\n');
end

%% ---- NN summary across seeds ----
fprintf('\n--- NN across seeds: mean +/- std ---\n');
nn = results(strcmp(results.Controller,'NN'), :);
for sname = unique(nn.Scenario, 'stable')'
    rows = nn(strcmp(nn.Scenario, sname{1}), :);
    fprintf('\n  %s  (n = %d seeds)\n', sname{1}, height(rows));
    for mc = [metric_cols, {'RiseTime_s','DisturbanceDip_V','RecoveryTime_s','RecoveryOvershoot_pct'}]
        if ~ismember(mc{1}, rows.Properties.VariableNames), continue, end
        x = rows.(mc{1}); x = x(~isnan(x));
        if isempty(x), continue, end
        fprintf('    %-24s %10.5f +/- %-10.5f (min %.5f, max %.5f)\n', ...
                mc{1}, mean(x), std(x), min(x), max(x));
    end
end

if isfile([WEIGHTS_TARGET '.bak'])
    copyfile([WEIGHTS_TARGET '.bak'], WEIGHTS_TARGET, 'f');
    fprintf('\nRestored original %s\n', WEIGHTS_TARGET);
end


%% ============================================================
function row = compute_metrics_row(ctrl_name, sc, t, y, setpoint_vec, band_frac)
% Metrics for one logged response.
% IAE, ISE, SteadyStateError_V, Overshoot_pct and SettlingTime_s are
% computed exactly as in closed_loop_batch_simulation_2.m so that the
% numbers remain comparable with the earlier results. The remaining
% columns describe the disturbance transient and are NaN when the load
% step is not separated in time from the setpoint step.

    setpoint_vec = setpoint_vec(:);
    err = setpoint_vec - y;

    IAE = trapz(t, abs(err));
    ISE = trapz(t, err.^2);

    tail_idx = t >= (t(end) - 0.1*(t(end)-t(1)));
    steady_state_error = mean(abs(err(tail_idx)));

    if sc.setpoint ~= 0
        overshoot_pct = max(0, (max(y) - sc.setpoint) / sc.setpoint * 100);
    else
        overshoot_pct = NaN;
    end

    band = band_frac * abs(sc.setpoint);
    within = abs(err) <= band;
    idx = find(~within, 1, 'last');
    if isempty(idx)
        settling_time = t(1);
    elseif idx == numel(t)
        settling_time = NaN;
    else
        settling_time = t(idx + 1);
    end

    % ---- additional transient descriptors ----
    rise_time = NaN; dip = NaN; recovery = NaN; rec_overshoot = NaN;

    if sc.t_load > sc.t_step
        pre = t >= sc.t_step & t < sc.t_load;
        tp = t(pre); yp = y(pre);
        i10 = find(yp >= 0.1*sc.setpoint, 1);
        i90 = find(yp >= 0.9*sc.setpoint, 1);
        if ~isempty(i10) && ~isempty(i90), rise_time = tp(i90) - tp(i10); end

        post = t >= sc.t_load;
        tq = t(post); yq = y(post);
        dip = sc.setpoint - min(yq);
        rec_overshoot = max(0, (max(yq) - sc.setpoint) / sc.setpoint * 100);
        idx2 = find(abs(yq - sc.setpoint) > band, 1, 'last');
        if isempty(idx2)
            recovery = 0;
        elseif idx2 < numel(tq)
            recovery = tq(idx2 + 1) - sc.t_load;
        end
    end

    row = table({ctrl_name}, {sc.label}, sc.setpoint, sc.iload, ...
                sc.t_step, sc.t_load, ...
                IAE, ISE, steady_state_error, overshoot_pct, settling_time, ...
                rise_time, dip, recovery, rec_overshoot, ...
                'VariableNames', {'Controller','Scenario','Setpoint_V','Iload_A', ...
                'StepTime_s','LoadTime_s','IAE','ISE','SteadyStateError_V', ...
                'Overshoot_pct','SettlingTime_s','RiseTime_s','DisturbanceDip_V', ...
                'RecoveryTime_s','RecoveryOvershoot_pct'});
end
