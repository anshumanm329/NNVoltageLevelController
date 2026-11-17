function run_one_seed_2(seed)
% RUN_ONE_SEED Closed loop validation of ONE trained network
% run_one_seed(48)
% Why one seed per call: nn_controller_3.m holds its weights in persistent
% variables filled by coder.load() and simulink caches the code generated
% for that block. Inside one MATLAB session, these caches can survive close
% and clear functions and even deletion of slprj, so several seeds silently run the same network
% A fresh matlab process shares nothing with the previous run, which
% removes the problem rather than working around. 
%% ========================================================
% CONFIG
% =========================================================
model_name = 'Working_with_RLC_3_5';
target_block = [model_name '/Target'];
iload_block = [model_name '/Iload'];

WEIGHTS_TARGET = 'nn_weights_for_matlab_3.mat'; % name hard-coded in nn_controller_3.m
WEIGHTS_DIR = 'nn_seed_weights';

LOG_TIME_VAR = 'tout';
LOG_XDATA_VAR = 'X_data';
XDATA_SETPOINT_COL = 1;
XDATA_OUTPUT_COL = 2;
LOG_MPC_OUTPUT_VAR = 'simout_mpc_output';
LOG_PID_OUTPUT_VAR = 'simout_pid_output';
LOG_PID_FF_OUTPUT_VAR = 'simout_pid_ff_output';

StopTime = 0.1;
SETTLE_BAND_FRAC = 0.02;
STEP_TIME = 0.01;
LOAD_TIME = 0.04;
TRACE_DIR = 'closed_loop_traces';

scenarios = struct('label', {}, 'setpoint', {}, 'iload', {});
scenarios(end+1) = struct('label', 'on-grid_10V_0.2A', 'setpoint', 10, 'iload', 0.2);
scenarios(end+1) = struct('label', 'on-grid_12V_0.5A', 'setpoint', 12, 'iload', 0.5);
scenarios(end+1) = struct('label', 'ood_15V_0.3A', 'setpoint', 15, 'iload', 0.3);
scenarios(end+1) = struct('label', 'ood_22V_0.15A', 'setpoint', 22, 'iload', 0.15);
scenarios(end+1) = struct('label', 'novel_offgrid_9.4V_0.37A', 'setpoint', 9.4, 'iload', 0.37);
scenarios(end+1) = struct('label', 'paper_19V_0.3A', 'setpoint', 19, 'iload', 0.3);
%% =============================================================================
evalin('base', 'load(''mpc_with_fixed_gain.mat'')');
src = fullfile(WEIGHTS_DIR, sprintf('nn_weights_seed%d.mat',seed));
if ~isfile(src)
    error('Missing weights file: %s', src);
end
if isfile(WEIGHTS_TARGET) && ~isfile([WEIGHTS_TARGET '.bak'])
    copyfile(WEIGHTS_TARGET, [WEIGHTS_TARGET, '.bak']);
end
% Restore the original weights however this function ends - normal return,
% an error in a simulation, or Ctrl+C. The restore must not depend on the
% rest of the script succeeding.
restore_guard = onCleanup(@() local_restore(WEIGHTS_TARGET)); %#ok<NAGSU>

% -------- 1. delete Simulink's code-generation cache---------------------
% The code generated for the MATLAB Function block has the weights from
% coder.load baked in, and it is cached ON DISK (slprj/ and <model>.slxc)
% so it survives from one Matlab process to next. The cache may live in
% the working folder or in the CacheFolder set in Simulink preferences, so
% clear both places.....
cache_dirs = unique({pwd, Simulink.fileGenControl('get', 'CacheFolder')});
for i = 1:numel(cache_dirs)
    d = cache_dirs{i};
    if isfolder(fullfile(d, 'slprj'))          % only if it exists
        rmdir(fullfile(d, 'slprj'), 's');
    end
    if isfile(fullfile(d, [model_name '.slxc']))   % same for the .slxc
        delete(fullfile(d, [model_name '.slxc']));
    end
end
% ----------- 2. install this seed's weights with FRESH timestamp --------
% copyfile keeps the source file's timestamp and all the seed files
% share....
% unchanged from one seed to the next. Loading and re-saving writes a new
% file with a new timestamp.
W_seed = load(src);
save(WEIGHTS_TARGET, '-struct', 'W_seed');
% the network this process is supposed to run, kept for the check below
net = local_load_net(WEIGHTS_TARGET);
fprintf('seed %d: installed %s, sum(W1) = %+.10e\n', seed, src, sum(net.W{1}(:)));
% every network that could possibly be running: all seed files plus the
% original weights (.bak). The check below asks which of them explains the
% simulated steady state
f = dir(fullfile(WEIGHTS_DIR, 'nn_weights_seed*.mat'));
cand_seed = zeros(1, numel(f));
cand_nets = cell(1, numel(f));
for k = 1:numel(f)
    cand_seed(k) = sscanf(f(k).name, 'nn_weights_seed%d.mat');
    cand_nets{k} = local_load_net(fullfile(WEIGHTS_DIR, f(k).name));

end
if isfile([WEIGHTS_TARGET '.bak'])
    cand_seed(end+1) = -1; % -1 = original network
    cand_nets{end+1} = local_load_net([WEIGHTS_TARGET '.bak']);
end
if ~isfolder(TRACE_DIR), mkdir(TRACE_DIR); end
load_system(model_name);
results = table();

for s = 1:numel(scenarios)
    sc = scenarios(s);

    set_param(target_block, 'Time', num2str(STEP_TIME), 'Before', '0', ...
        'After', num2str(sc.setpoint));
    set_param(iload_block, 'Time', num2str(LOAD_TIME), 'Before', '0', ...
        'After', num2str(sc.iload));
    simOut = sim(model_name);

    t = simOut.get(LOG_TIME_VAR); t = t(:);
    X = simOut.get(LOG_XDATA_VAR);
    sp_vec = X(:, XDATA_SETPOINT_COL);

    y_nn = X(:, XDATA_OUTPUT_COL);
    y_mpc = simOut.get(LOG_MPC_OUTPUT_VAR); y_mpc = y_mpc(:);
    y_pid = simOut.get(LOG_PID_OUTPUT_VAR); y_pid = y_pid(:);
    y_pid_ff = simOut.get(LOG_PID_FF_OUTPUT_VAR); y_pid_ff = y_pid_ff(:);

    % --------- 3. verify that the installed network is the one that ran---
    % At steady state the plant satisfies Vout = 4.8*u - 10*Iload with
    % IL = 0 and Vc = Vout + 10*Iload (checked against the training data). 
    % Feed the simulated steady state into the installed network: if that
    % network really produced the response, the equation closes to ~1e-6 V.
    % A different network misses it by 0.01 - 0.2 V.
    y_ss = mean(y_nn(t >= StopTime - 0.01));
    x_ss = [sc.setpoint, repmat(y_ss, 1, 6), y_ss + 10*sc.iload, 0, sc.iload];
    res = zeros(1, numel(cand_nets));
    for k = 1:numel(cand_nets)
        res(k) = abs(4.8*local_net_eval(cand_nets{k}, x_ss) - 10*sc.iload - y_ss);

    end
    residual = res(cand_seed == seed);
    [~, best] = min(res);
    others = res(cand_seed ~= seed);
    fprintf(['%-26s sum(Vout_NN) = %+.10e residual = %.2eV'...
        '(nearest other network: %.2e V)\n'], ...
        sc.label, sum(y_nn), residual, min(others));
    if cand_seed(best) ~= seed || residual > 1e-3
        error(['STALE NETWORK (seed %d, %s): the response is explained by' ...
            'seed %d (-1 = original weights), residual of the installed' ...
            'seed %.3g V. These results must not be used.'], ...
            seed, sc.label, cand_seed(best), residual);
    end
    
    names = {'NN','MPC','PID','PID_with_FF'};
    sigs  = {y_nn, y_mpc, y_pid, y_pid_ff};
    for c = 1:4
         row = local_metrics_row(names{c}, sc, t, sigs{c}, sp_vec, ...
                                SETTLE_BAND_FRAC, STEP_TIME, LOAD_TIME, StopTime);
         row.seed = seed;
         results = [results; row]; %#ok<AGROW>
    end
    trace = table(t, sp_vec, y_nn, y_mpc, y_pid, y_pid_ff, ...
         'VariableNames', {'time_s', 'setpoint_V', 'Vout_NN', 'Vout_MPC_V', ...
         'Vout_PID_V', 'Vout_PID_FF_V'});
    writetable(trace, fullfile(TRACE_DIR, sprintf('trace_seed%d_%s.csv', ...
         seed, matlab.lang.makeValidName(sc.label))));
end
results = movevars(results, 'seed', 'Before', 1);
out_csv = sprintf('results_seed%d.csv', seed);
writetable(results, out_csv);
fprintf('seed %d: wrote %s and %d traces\n', seed, out_csv, numel(scenarios));

end

%% ========================================================================
function net = local_load_net(path)
    % Read the weights in the same layout nn_controller_3.m uses.
    m = load(path, '-mat');
    L = double(m.num_layers);
    net.W = cell(1, L); net.b = cell(1,L);
    for i = 1:L
        net.W{i} = m.(sprintf('W%d', i));
        net.b{i} = m.(sprintf('b%d', i));

    end
    net.mu = m.input_mean(:)'; net.sd = m.input_std(:)';
    net.om = double(m.output_mean); net.osd = double(m.output_std);
    net.umin = double(m.u_min); net.umax = double(m.u_max);
end
%% ========================================================================
function u = local_net_eval(net, x)
    z = (x - net.mu) ./net.sd;
    L = numel(net.W);
    for i = 1:L
        z = z * net.W{i} + net.b{i};
        if i < L, z = max(z,0); end
    end
    u = min(max(z*net.osd + net.om, net.umin), net.umax);
end
%% ========================================================================
function local_restore(target)
    bak = [target, '.bak'];
    if isfile(bak)
        [ok, msg] = copyfile(bak, target, 'f');
        if ok
            w = load(target, 'W1');
            fprintf('restored original %s, sum(W1) = %+.10e\n', target, sum(w.W1(:)));
        else
            fprintf(2, 'WARNING: could not restore %s from %s: %s\n', target, bak, msg);
        end
    end
end

%% ========================================================================
function row = local_metrics_row(ctrl, sc, t, y, sp_vec, band_frac, t_step, ...
    t_load, t_stop)
    sp = sc.setpoint;
    err = sp_vec(:) - y;
    tol = band_frac * abs(sp);
    IAE = trapz(t, abs(err));
    ISE = trapz(t, err.^2);

    pre = t >= t_step & t < t_load ; tp = t(pre); yp = y(pre);
    overshoot = max(0, (max(yp) - sp)/sp*100);
    idx = find(abs(yp-sp) > tol, 1, 'last');
    if isempty(idx), settling = 0;
    elseif idx < numel(tp), settling = tp(idx+1) - t_step;
    else, settling = NaN;
    end

    i10 = find(yp >= 0.1*sp, 1); i90 = find(yp >= 0.9*sp,1);
    if ~isempty(i10) && ~isempty(i90), rise = tp(i90) - tp(i10);
    else, rise = NaN; end
    tail = tp >= t_load - 0.2*(t_load - t_step);
    sse = mean(abs(yp(tail)-sp));

    post = t >= t_load; tq = t(post); yq = y(post);
    dip = sp - min(yq);
    rec_os = max(0, (max(yq)-sp)/sp*100);
    idx2 = find(abs(yq-sp) > tol, 1, 'last');
    if isempty(idx2), recovery = 0;
    elseif idx2 < numel(tq), recovery = tq(idx2+1) - t_load;
    else, recovery = NaN;
    end
    tail2 = tq >= t_stop - 0.2*(t_stop - t_load);
    sse_after = mean(abs(yq(tail2) - sp));

    row = table({ctrl}, {sc.label}, sp, sc.iload, t_step, t_load, ...
            IAE, ISE, sse, overshoot, settling, rise, dip, recovery, rec_os, sse_after, ...
            'VariableNames', {'Controller','Scenario','Setpoint_V','Iload_A', ...
            'StepTime_s','LoadTime_s','IAE','ISE','SteadyStateError_V', ...
            'Overshoot_pct','SettlingTime_s','RiseTime_s','DisturbanceDip_V', ...
            'RecoveryTime_s','RecoveryOvershoot_pct','SteadyStateError_after_V'});
end

        
% To combine afterwards, in MATLAB:
%   f = dir('results_seed*.csv');
%   T = table();
%   for i = 1:numel(f), T = [T; readtable(f(i).name)]; end
%   writetable(T, 'seed_sweep_closedloop_results.csv');





    

