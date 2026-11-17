%% run_all_seeds_2.m
% Runs run_one_seed_2.m for every seed, each in its own fresh matlab process,
% holds the model, nn_controller_3.m, mpc_with_fixed_gain.mat,
% run_one_seed.m and nn_seed_weights folder

clear; clc;
SEEDS = [48 1 2 3 4 5 6 7 8 9];

if ispc
    mexe = fullfile(matlabroot, 'bin', 'matlab.exe');
else
    mexe = fullfile(matlabroot, 'bin', 'matlab');
end
workdir = pwd;

log = strings(numel(SEEDS), 1);
for k = 1:numel(SEEDS)
    s = SEEDS(k);
    fprintf('\n======== seed %d (%d/%d): starting a fresh MATLAB process =========\n', ...
        s, k, numel(SEEDS));
    % the child process starts in its default folder....
    batch_cmd = sprintf('cd(''%s''); run_one_seed_2(%d)', workdir, s);
    cmd = sprintf('"%s" -batch "%s"', mexe, batch_cmd);
    tic;
    [status, out] = system(cmd);
    fprintf('%s', out); % echo the child's output
    fprintf('seed %d finished in %.0f seconds, exit status %d\n', s, toc, status);
    if status ~= 0
       warning('seed %d failed -see output above. Continuing with the rest.', s);
    end
    log(k) = string(out);
end

%% --------- combine the per-seed tables -------
f = dir('results_seed*.csv');
T = table();
for i = 1:numel(f)
    T = [T; readtable(f(i).name)]; %#ok<AGROW>
end
writetable(T, 'seed_sweep_closedloop_results.csv');
fprintf('\nCombined %d per-seed files into seed_sweed_closedloop_results.csv (%d rows)\n', ...
    numel(f), height(T));
%% ------- quick check: did each seed run its own network? -------
nn = T(strcmp(T.Controller, 'NN') & strcmp(T.Scenario, 'paper_19V_0.3A'), :);
fp = round(nn.IAE, 12);
fprintf('distinct NN responses a 19V / 0.3A %d of %d seeds\n', ...
    numel(unique(fp)), height(nn));
if numel(unique(fp)) < height(nn)
    warning('Some seeds produced identical NN responses - check before using these results.');
end


