function v_control = nn_controller_3(setpoint, plant_output, plant_output_t1,...
                                     plant_output_t2, plant_output_t3, ...
                                     plant_output_t4, plant_output_t5, ...
                                     Vc, IL, Iload)
%% Function block to import the weights and biases of a trained neural network
%   The exported weights are to be taken from nn_weights_for_matlab_3.mat
%   The inputs have to be passed in exactly the same order (Must match the
%   list Features as generated in Python....
%   i.e., Vin, Vout, Vout (t-1)...Vout(t-5), Vc, IL, Iload

%% Define the required variables for the function as persistent...
persistent W1 W2 W3 W4 W5 W6 b1 b2 b3 b4 b5 b6 act1 act2 act3 act4 act5 act6 ...
           input_mean input_std ...
           output_mean output_std U_min U_max num_layers

%% 1. Load the required data
if isempty(num_layers)
    % load the nn weights from matlab
    loaded_data = coder.load('nn_weights_for_matlab_3.mat');
    num_layers = 6;
    
    
    W1 = loaded_data.W1; b1 = loaded_data.b1; act1 = char(loaded_data.activation1);
    W2 = loaded_data.W2; b2 = loaded_data.b2; act2 = char(loaded_data.activation2);
    W3 = loaded_data.W3; b3 = loaded_data.b3; act3 = char(loaded_data.activation3);
    W4 = loaded_data.W4; b4 = loaded_data.b4; act4 = char(loaded_data.activation4);
    W5 = loaded_data.W5; b5 = loaded_data.b5; act5 = char(loaded_data.activation5);
    W6 = loaded_data.W6; b6 = loaded_data.b6; act6 = char(loaded_data.activation6);
    input_mean = loaded_data.input_mean;
    input_std = loaded_data.input_std;
    output_mean = loaded_data.output_mean;
    output_std = loaded_data.output_std;
    U_min = loaded_data.u_min;
    U_max = loaded_data.u_max;

end
%% 2. Assemble the input row vector in the exact training order
features = [setpoint, plant_output, plant_output_t1, ...
            plant_output_t2, plant_output_t3, ...
            plant_output_t4, plant_output_t5, ...
            Vc, IL, Iload];

%%% 2.1. Normalize in the same way as normalization was done during
%%% training
features_norm = (features - input_mean) ./ input_std;

%% 3. The forward pass....
h1 = features_norm * W1 + b1;
if strcmp(act1, 'relu'), h1 = max(h1, 0);
elseif ~strcmp(act1, 'linear'), error('Unsupported activation: %s', act1');
end

h2 = h1 * W2 + b2;
if strcmp(act2, 'relu'), h2 = max(h2, 0);
elseif ~strcmp(act2, 'linear'), error('Unsupported activation: %s', act2');
end

h3 = h2 * W3 + b3;
if strcmp(act3, 'relu'), h3 = max(h3, 0);
elseif ~strcmp(act3, 'linear'), error('Unsupported activation: %s', act3');
end

h4 = h3 * W4 + b4;
if strcmp(act4, 'relu'), h4 = max(h4, 0);
elseif ~strcmp(act4, 'linear'), error('Unsupported activation: %s', act4);
end

h5 = h4 * W5 + b5;
if strcmp(act5, 'relu'), h5 = max(h5, 0);
elseif ~strcmp(act5, 'linear'), error('Unsupported activation: %s', act5);
end

out = h5 * W6 + b6;
if strcmp(act6, 'relu'), out = max(out, 0);
elseif ~strcmp(act6, 'linear'), error('Unsupported activation: %s', act6);
end

v_control_raw = out * output_std + output_mean;

%% Zero-setpoint safety check
if features(1) < 0.1
    v_control_raw = 0.0;
    v_control = v_control_raw;
    return;
end

%% De-normalize the output and then clip it to the actuator range...
v_control = min(max(v_control_raw, U_min), U_max);

end