% Define common parameters
C = 1e-9; % Farads
L = 1e-3; % Henrys

% Time vector for simulation
t = linspace(0, 0.0002, 1000);

% Overdamped case (R^2 > 4L/C)
R_overdamped = 500; % Ohms
A_overdamped = [1];
B_overdamped = [L, R_overdamped, 1/C];
sys_overdamped = tf(A_overdamped, B_overdamped);

% Critically damped case (R^2 = 4L/C)
R_critically_damped = 200; % Ohms
A_critically_damped = [1];
B_critically_damped = [L, R_critically_damped, 1/C];
sys_critically_damped = tf(A_critically_damped, B_critically_damped);

% Underdamped case (R^2 < 4L/C)
R_underdamped = 50; % Ohms
A_underdamped = [1];
B_underdamped = [L, R_underdamped, 1/C];
sys_underdamped = tf(A_underdamped, B_underdamped);

% Plot the step responses
figure;

% Overdamped response
subplot(3, 1, 1);
step(sys_overdamped, t);
title('Overdamped Response');
xlabel('Time (s)');
ylabel('Response');
grid on;

% Critically damped response
subplot(3, 1, 2);
step(sys_critically_damped, t);
title('Critically Damped Response');
xlabel('Time (s)');
ylabel('Response');
grid on;

% Underdamped response
subplot(3, 1, 3);
step(sys_underdamped, t);
title('Underdamped Response');
xlabel('Time (s)');
ylabel('Response');
grid on;

% Adjust the layout
sgtitle('Step Responses of Overdamped, Critically Damped, and Underdamped Systems');
