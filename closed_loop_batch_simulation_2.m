%% closed_loop_validation_batch.m
%
% Runs Working_with_RLC across a set of test scenarios, for both the NN
% and MPC controllers (PID section marked TODO - see note below), logs
% the closed-loop response, and computes a consistent metrics table:
% IAE, ISE, settling time, overshoot, and steady-state error.
%
% This follows the same batch-run pattern as your data-generation /
% DAgger scripts: set block parameters programmatically, run sim, log via
% To Workspace, move to the next scenario.
 
clear; clc;
 
%% ============================================================
%  CONFIG - edit to match your model's exact block/variable names
%  ============================================================
 
load('mpc_with_fixed_gain.mat'); % Load the mpc model
load('nn_weights_for_matlab_3.mat'); % Load the nn model that we have worked out
model_name = 'Working_with_RLC_3_5';

% Block paths - EDIT these to match your actual model hierarchy.
% (Use e.g. get_param(model_name,'Handle') + find_system to look these up
%  if unsure: find_system(model_name,'BlockType','ManualSwitch'))
target_block   = [model_name '/Target'];        % Step block for setpoint
iload_block    = [model_name '/Iload'];         % Step block for disturbance
% switch_block   = [model_name '/Manual Switch1']; % NN=1, MPC=2
 
% Logged signal names as they land in the workspace after sim.
% - Time: enable Time logging in Configuration Parameters -> Data
%   Import/Export (variable name 'tout') - required, no tout = no way to
%   pair samples with a time axis for trapz().
% - Switch-selected (NN/MPC) plant output: NOT a separate block - it's
%   already column 2 of out.X_data (plant_output(t)), per your training
%   feature layout [setpoint, plant_output(t), plant_output(t-1..t-5),
%   Vc, IL, Iload]. Column 1 is the logged setpoint, used below instead
%   of assuming a constant.
% - PID output: separate To Workspace block you're adding now, logging
%   the PID plant's plant_output (Vc - Iload*Rload, same definition as
%   X_data column 2) so all three controllers are compared on the same
%   quantity.
LOG_TIME_VAR   = 'tout';
LOG_XDATA_VAR  = 'X_data';            % struct/array with cols [setpoint, plant_output(t), ...]
XDATA_SETPOINT_COL = 1;
XDATA_OUTPUT_COL   = 2;
LOG_MPC_OUTPUT_VAR = 'simout_mpc_output';
LOG_PID_OUTPUT_VAR = 'simout_pid_output';  
LOG_PID_FF_OUTPUT_VAR = 'simout_pid_ff_output';                                            % name the new PID To Workspace block
 
StopTime = 0.1;    % match your data-generation script's StopTime
SETTLE_BAND_FRAC = 0.02;  % +/-2% band for settling time, adjust if your
                           % existing settling-time calc uses a different band
 
%% ============================================================
%  Test scenarios: on-grid, held-out OOD, and one novel off-grid point
%  ============================================================
% EDIT: replace with your actual chosen values. OOD setpoints are 15, 22
% per your training split. "on-grid" here means points that exist in your
% densified training grid; "novel off-grid" means a value that is NOT on
% the densified grid at all (not just an OOD setpoint on-grid).
 
scenarios = struct( ...
    'label',    {}, 'setpoint', {}, 'iload', {} ...
);
scenarios(end+1) = struct('label','on-grid_10V_0.2A',  'setpoint',10, 'iload',0.2);
scenarios(end+1) = struct('label','on-grid_12V_0.5A',  'setpoint',12, 'iload',0.5);
scenarios(end+1) = struct('label','ood_15V_0.3A',      'setpoint',15, 'iload',0.3);
scenarios(end+1) = struct('label','ood_22V_0.15A',     'setpoint',22, 'iload',0.15);
scenarios(end+1) = struct('label', 'paper_19V_0.3A', 'setpoint', 19, 'iload', 0.3);
scenarios(end+1) = struct('label','novel_offgrid_9.4V_0.37A', 'setpoint',9.4, 'iload',0.37);
 
% controllers = struct('name', {'NN','MPC'}, 'switch_val', {'1','2'});
% PID is NOT part of this switch - it's a parallel plant replica in the
% same model, sharing the Target/Iload blocks, so its output is logged
% every run regardless of switch position. Handled once per scenario
% below (not duplicated across the NN/MPC loop).
 
%% ============================================================
%  Load model once
%  ============================================================
load_system(model_name);
 
results = table();
 
for s = 1:numel(scenarios)
    sc = scenarios(s);
 
    % ---- Set scenario parameters once per scenario (shared by both
    % the switch-selected branch and the PID branch) ----
    set_param(target_block, 'Time', '0', 'Before', '0', ...
               'After', num2str(sc.setpoint));
    set_param(iload_block, 'Time', '0', 'Before', '0', ...
               'After', num2str(sc.iload));
    set_param(model_name, 'StopTime', num2str(StopTime));
    fprintf('Running %s| Setpoint: %.2f,V Iload: %.2f A\n', sc.label, sc.setpoint, sc.iload);
    simOut = sim(model_name);
    t = simOut.get(LOG_TIME_VAR);
    t = t(:);
    X = simOut.get(LOG_XDATA_VAR);
    y_nn = X(:, XDATA_OUTPUT_COL);
    setpoint_logged = X(:, XDATA_SETPOINT_COL);
    
    row_nn = compute_metrics_row('NN', sc, t, y_nn, setpoint_logged);
    results = [results; row_nn]; %#ok<AGROW>
    % ----- MPC metrics ------ %
    y_mpc = simOut.get(LOG_MPC_OUTPUT_VAR);
    y_mpc = y_mpc(:);
    row_mpc = compute_metrics_row('MPC', sc, t, y_mpc, setpoint_logged);
    results = [results; row_mpc];
    % ---- PID metrics: only record once per scenario (its output
    y_pid = simOut.get(LOG_PID_OUTPUT_VAR);
    y_pid = y_pid(:);
    % PID branch has no separate logged setpoint - it shares the
    % same Target block as the switch-selected plant, so the
    % setpoint_logged vector pulled from X_data above applies
    % here too (same run, same Target step).
    row_pid = compute_metrics_row('PID', sc, t, y_pid, setpoint_logged);
    results = [results; row_pid]; %#ok<AGROW>
    % PID_FF branch has no separate logged setpoint - it shares the
    % same Target block as the switch-selected plant, so the
    % setpoint_logged vector pulled from X_data above applies
    % here too (same run, same Target step).
    y_pid_ff = simOut.get(LOG_PID_FF_OUTPUT_VAR);
    y_pid_ff = y_pid_ff(:);
    row_pid_ff = compute_metrics_row('PID_with_FF', sc, t, y_pid_ff, setpoint_logged);
    results = [results; row_pid_ff];
end
 
disp(results);
writetable(results, 'closed_loop_validation_results_2.csv');
fprintf('\nSaved results to closed_loop_validation_results_2.csv\n');
 
 
%% ============================================================
%  Local function: compute IAE/ISE/settling time/overshoot/SSE for one
%  logged run and return it as a one-row table
%  ============================================================
function row = compute_metrics_row(ctrl_name, sc, t, y, setpoint_vec)
    SETTLE_BAND_FRAC = 0.02;  % keep in sync with the value near CONFIG above
 
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
 
    band = SETTLE_BAND_FRAC * abs(sc.setpoint);
    within_band = abs(err) <= band;
    settle_idx = find(~within_band, 1, 'last');
    if isempty(settle_idx)
        settling_time = t(1);
    elseif settle_idx == length(t)
        settling_time = NaN;
    else
        settling_time = t(settle_idx + 1);
    end
 
    row = table({ctrl_name}, {sc.label}, sc.setpoint, sc.iload, ...
                 IAE, ISE, steady_state_error, overshoot_pct, settling_time, ...
                 'VariableNames', {'Controller','Scenario','Setpoint_V', ...
                 'Iload_A','IAE','ISE','SteadyStateError_V', ...
                 'Overshoot_pct','SettlingTime_s'});
end
