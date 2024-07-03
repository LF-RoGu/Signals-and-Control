% Define the transfer function coefficients
num = [1 -1]; % Numerator coefficients
den = [1 2 1]; % Denominator coefficients

% Create the transfer function system
system = tf(num, den);

% Plot the step response using the built-in step function
figure;
step(system);
title('Step Response using Built-in step Function');
xlabel('Time [s]');
ylabel('Response');
grid on;
