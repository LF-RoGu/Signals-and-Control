% Given values
m1 = 0.3; % kg
m2 = 0.1; % kg
k1 = 1; % N/m
k2 = 1; % N/m
b1 = 0.1; % kg/s
b2 = 0.1; % kg/s
% State-space matrices
A = [0, 1, 0, 0;
-(k1 + k2)/m1, 0, k2/m1, 0;
0, 0, 0, 1;
k2/m2, 0, -k2/m2, 0];
B = [0; 1/m1; 0; 0];
C = [0, 0, 1, 0];
D = 0;
% Create state-space system
sys = ss(A, B, C, D);
% Plot the step response
figure;
step(sys);
title('Step Response');
xlabel('Time (s)');
ylabel('Output q(t)');
grid on;
% Plot the frequency response
figure;
bode(sys);
title('Frequency Response');
grid on;
% Simulate the response to a sine wave
t = 0:0.01:20; % Time vector
u1 = sin(2*pi*0.5*t); % Sine wave with frequency 0.5 Hz and amplitude 1 N
u2 = sin(2*pi*0.2*t); % Sine wave with frequency 0.2 Hz and amplitude 1 N
% Response to 0.5 Hz sine wave
figure;
lsim(sys, u1, t);
title('Response to 0.5 Hz Sine Wave');
xlabel('Time (s)');
ylabel('Output q(t)');
grid on;
% Response to 0.2 Hz sine wave
figure;
lsim(sys, u2, t);
title('Response to 0.2 Hz Sine Wave');
xlabel('Time (s)');
ylabel('Output q(t)');
grid on;