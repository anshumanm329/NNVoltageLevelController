% This is a program to capture the computation time for the MPC and NN
% Run 'closed_loop_batch_simulation_2.m' before ruuning this script,
% otherwise it will NOT WORK. 

n_samples = 50; 
% NN vs MPC per control step 
% Measurement of wall clock time to compute one control action 
%% --------------------------------------------------------------%
% STEP - 1: Load sample operating points from a real closed run
% --------------------------------------------------------------%
% The states being recorded are: 
% Vc, IL (The two plant states)
% Vout, Vout (t-1)...(t-5): For NN's feature vectors
% setpoint (reference)
% Iload (measured disturbance)
% prev_mv (previous control action, needed mpcmove)
X = simOut.get('X_data'); % It is indeed the logged variable for the NN
nn_mv_log = simOut.get('nn_mv_log'); % The NN controller action
% Pickout n_samples = 50 evenly spaced across the run, skipping the very
% first sample and avoiding the last few
N = size(X,1); 
idx = round(linspace(2, N-1, n_samples));

setpoint_samples = X(idx, 1);
vout_samples = X(idx, 2);
vout_delay_samples = X(idx, 3:7); % Vout(t-1)...(t-5)
vc_samples = X(idx, 8); 
il_samples = X(idx, 9);
iload_samples = X(idx, 10);
prev_mv_samples = nn_mv_log(idx-1); %NN's own action from the previous step

%% -----------------------------------------------------------------------
% STEP-2: Time the NN forward pass
% ------------------------------------------------------------------------
% EDIT: Load your exported weights
loaded_data = load('nn_weights_for_matlab_3.mat'); 
W = {loaded_data.W1, loaded_data.W2, loaded_data.W3, loaded_data.W4, loaded_data.W5, loaded_data.W6};
b = {loaded_data.b1, loaded_data.b2, loaded_data.b3, loaded_data.b4, loaded_data.b5, loaded_data.b6};
act = {loaded_data.activation1, loaded_data.activation2, ...
    loaded_data.activation3, loaded_data.activation4, ...
    loaded_data.activation5, loaded_data.activation6};
input_mean = loaded_data.input_mean;
input_std = loaded_data.input_std;
output_mean = loaded_data.output_mean;
output_std = loaded_data.output_std;
U_min = loaded_data.u_min;
U_max = loaded_data.u_max;
nn_times = zeros(n_samples, 1);
nn_forward([setpoint_samples(1), vout_samples(1), vout_delay_samples(1,:), vc_samples(1), il_samples(1), iload_samples(1)], ...
           W, b, act, input_mean, input_std, output_mean, output_std, U_min, U_max);

for i = 1:n_samples
    raw_feature = [setpoint_samples(i), vout_samples(i), vout_delay_samples(i,:), ...
        vc_samples(i), il_samples(i), iload_samples(i)];
    f = @() nn_forward(raw_feature, W, b, act, input_mean, input_std, output_mean, output_std, U_min, U_max);
    nn_times(i) = timeit(f);
end
nn_mean_ms = mean(nn_times)*1000;
nn_std_ms = std(nn_times)*1000;
fprintf('NN forward pass: %.6f +/- %.6f ms per step\n', ...
    nn_mean_ms, nn_std_ms);

%% ------------------------------------------------------------------------
% STEP-3: Time the MPC solve
% -------------------------------------------------------------------------
% mpc_controller_3 must already exist in the workspace
% if not carry out:
% load('mpc_with_fixed_gain.mat');
mpc_times = zeros(n_samples, 1);
mpc_iterations = zeros(n_samples, 1);
mpc_vc = simOut.get('simout_mpc_vc');
mpc_il = simOut.get('simout_mpc_il');
mpc_vout = simOut.get('simout_mpc_output');
mpc_vout_samples = mpc_vout(idx, 1);
mpc_vc_samples = mpc_vc(idx, 1);
mpc_il_samples = mpc_il(idx, 1);
% result discarded, runs once before any timing starts ---
xmpc_warmup = mpcstate(mpc_controller_3);
xmpc_warmup.Plant = [mpc_vc_samples(1); mpc_il_samples(1)];
mpcmove(mpc_controller_3, xmpc_warmup, mpc_vout_samples(1), setpoint_samples(1), iload_samples(1));
for i = 1:n_samples
    xmpc = mpcstate(mpc_controller_3);
    xmpc.Plant = [mpc_vc_samples(i); mpc_il_samples(i)];

    y_meas = mpc_vout_samples(i);
    ref = setpoint_samples(i);
    md = iload_samples(i);

    f = @() mpcmove(mpc_controller_3, xmpc, y_meas, ref, md);
    mpc_times(i) = timeit(f);

    % Separately capture iteration count
    [~, info] = mpcmove(mpc_controller_3, xmpc, y_meas, ref, md);
    if isfield(info, 'Iterations')
        mpc_iterations(i) = info.Iterations;
    end
end

mpc_mean_ms = mean(mpc_times) * 1000;
mpc_std_ms = std(mpc_times) * 1000;
fprintf('MPC solve: %.6f +/- %.6f ms per step\n', mpc_mean_ms, mpc_std_ms);
if any(mpc_iterations > 0)
    fprintf('MPC QP iterations: %.2f +/- %.2f (mean+/- std)\n', ...
        mean(mpc_iterations), std(mpc_iterations));
end
fprintf('\n Speedup: (MPC time/ NN time): %.1fx\n', mpc_mean_ms / nn_mean_ms);

%% ------------------------------------------------------------------------
% STEP-4: FLOP count for the N forward pass (derived from actual loaded
% weights and biases)
total_flops = 0;
for k = 1:length(W)
    [in_dim, out_dim] = size(W{k});
    total_flops = total_flops + 2 * in_dim * out_dim;
end
fprintf('\nNN forward-pass FLOPs (derived from actual weight matrices, %d layers): %d\n', ...
    length(W), total_flops)
fprintf('(MPC FLOPs are data-dependent - report mean QP iterations above instead,\n');
fprintf(' each iteration solving a linear system scaling with the prediction/control horizon.)\n');



%% Define the function to compute the NN forward pass
function u = nn_forward(raw_feature, W, b, act, input_mean, input_std, output_mean, output_std, U_min, U_max)
 x = (raw_feature - input_mean) ./ input_std;
 
 h = x;
 for k = 1:length(W)
     h = h * W{k} + b{k};
     if strcmp(act{k}, 'relu')
         h = max(h, 0);
     elseif ~strcmp(act{k}, 'linear')
         error('Unsupported activation: %s', act{k});
     end
 end
 u_raw = h * output_std + output_mean;
 u = min(max(u_raw, U_min), U_max);
end


   
