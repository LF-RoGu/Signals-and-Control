% Parameters
M = 1;     % Mass
k = 1;     % Spring constant
b = 0.2;   % Damping coefficient

% State space representation
A = [0 1; -k/M -b/M];
B = [0; 1/M];
C = [1 0];
D = 0;

% Time vector
t = 0:0.01:50;  % 10 seconds simulation with time step of 0.01 seconds

% Input force (u)
u = ones(size(t));  % Step input of 1 N

% Initial conditions
x0 = [0; 0];  % Initial displacement and velocity

% Simulate the system
sys = ss(A, B, C, D);
[y, t, x] = lsim(sys, u, t, x0);

% Plot results
figure;
subplot(2,1,1);
plot(t, x(:,1));
title('Mass-Spring-Damper System Response');
xlabel('Time (s)');
ylabel('Displacement (m)');
grid on;

subplot(2,1,2);
plot(t, x(:,2));
xlabel('Time (s)');
ylabel('Velocity (m/s)');
grid on;

% Display state-space matrices
disp('State-space representation:');
disp('A = '); disp(A);
disp('B = '); disp(B);
disp('C = '); disp(C);
disp('D = '); disp(D);
