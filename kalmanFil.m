% Parameters
dt = 1; % Time step (seconds)
num_steps = 50; % Number of steps in the simulation

% State Vector [position; velocity]
x = [0; 20]; % Initial position = 0, velocity = 20 m/s

% State Transition Matrix
A = [1 dt; 0 1]; % State transition for constant velocity

% Control Matrix (no control input in this example)
B = [0; 0];

% Measurement Matrix
H = [1 0]; % Only measuring position

% Process Noise Covariance (how uncertain we are about the process model)
Q = [0.1 0; 0 0.1];

% Measurement Noise Covariance (how uncertain we are about the measurement)
R = 10;

% Initial Covariance Matrix (initial uncertainty)
P = eye(2) * 10;

% Storage for results
x_estimates = zeros(2, num_steps); % Store state estimates
x_true = zeros(2, num_steps); % Store true states
z_measurements = zeros(1, num_steps); % Store measurements

% True initial state
x_true(:, 1) = x;

% Kalman Filter Loop
for k = 1:num_steps
    % Generate true position with constant velocity
    x_true(:, k) = A * x_true(:, max(k-1,1));
    
    % Simulate a noisy measurement of position
    z = H * x_true(:, k) + sqrt(R) * randn;
    z_measurements(k) = z;

    % Prediction Step
    x_pred = A * x; % Predict state
    P_pred = A * P * A' + Q; % Predict covariance

    % Measurement Update Step
    K = P_pred * H' / (H * P_pred * H' + R); % Kalman Gain
    x = x_pred + K * (z - H * x_pred); % Update state estimate
    P = (eye(2) - K * H) * P_pred; % Update covariance estimate

    % Store the estimated state
    x_estimates(:, k) = x;
end

% Plot Results
figure;
subplot(2,1,1);
plot(1:num_steps, x_true(1,:), 'g-', 'LineWidth', 1.5); hold on;
plot(1:num_steps, z_measurements, 'r.', 'MarkerSize', 10);
plot(1:num_steps, x_estimates(1,:), 'b--', 'LineWidth', 1.5);
legend('True Position', 'Measurements', 'Estimated Position');
xlabel('Time Step');
ylabel('Position');
title('1D Kalman Filter - Position Estimation');

subplot(2,1,2);
plot(1:num_steps, x_true(2,:), 'g-', 'LineWidth', 1.5); hold on;
plot(1:num_steps, x_estimates(2,:), 'b--', 'LineWidth', 1.5);
legend('True Velocity', 'Estimated Velocity');
xlabel('Time Step');
ylabel('Velocity');
title('1D Kalman Filter - Velocity Estimation');
