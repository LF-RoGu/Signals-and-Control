% Define the parameters
omega_0 = 1; % Natural frequency
sigma_values = [1.5, 1.0, -1.5]; % Different values of sigma (damping ratio)

% Time vector for plotting
t = linspace(0, 10, 1000);

% Plot unit step response for different values of sigma
figure;
hold on;
grid on;
title('Unit Step Response for Different Values of \sigma');
xlabel('Time [s]');
ylabel('Response y(t)');

for sigma = sigma_values
    % Define system transfer function coefficients
    num = [omega_0^2];
    den = [1 2*sigma*omega_0 omega_0^2];
    
    % Create transfer function system
    system = tf(num, den);
    
    % Compute step response
    [y, t_out] = step(system, t);
    
    % Plot step response
    plot(t_out, y, 'DisplayName', ['\sigma = ', num2str(sigma)]);
end

legend;
hold off;
