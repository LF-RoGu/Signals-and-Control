% Define the transfer function
H = tf([1 -1], [1 2 2]);

% Define frequency range for Bode plot
omega = logspace(-1, 2, 1000); % from 0.1 to 100 rad/s

% Bode plot using built-in function
figure;
bode(H, omega);
title('Bode Plot of H(s) = (s - 1) / (s^2 + 2s + 2) using built-in bode function');
grid on;
