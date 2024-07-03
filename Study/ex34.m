% Define the time vector
t = linspace(0, 10, 1000);

% Define the impulse response manually
h_t = (1 - t) .* exp(-t);

% Plot the manually calculated impulse response
figure;
plot(t, h_t);
title('Impulse Response without Using Built-in impulse Function');
xlabel('Time [s]');
ylabel('Response');
grid on;
