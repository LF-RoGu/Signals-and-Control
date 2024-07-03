clear;
close all;
clc;

% Parameters of the BLDC motor
L = 0.001;  % Inductance (H)
R = 1;      % Resistance (Ohms)
Kb = 0.01;  % Back EMF constant (V/(rad/s))
Kt = 0.01;  % Torque constant (Nm/A)
J = 0.01;   % Rotor inertia (kg*m^2)
B = 0.001;  % Damping coefficient (Nms)

% PID controller gains
Kp = 300;
Ki = 10;
Kd = 50;

% Desired number of revolutions
desired_revolutions = 100;
desired_position = desired_revolutions * 2 * pi; % Convert to radians

% Simulation parameters
t_final = 60;  % Simulation time (s)
dt = 0.001;    % Time step (s)
t = 0:dt:t_final;

% Initial conditions
Ia = 0;      % Initial current (A)
omega = 0;   % Initial angular velocity (rad/s)
theta = 0;   % Initial position (rad)

% Preallocate arrays for results
Ia_arr = zeros(size(t));
omega_arr = zeros(size(t));
theta_arr = zeros(size(t));
error_arr = zeros(size(t));
control_signal_arr = zeros(size(t));

% PID controller variables
integral = 0;
previous_error = 0;

% Simulation loop
for i = 1:length(t)
    % Calculate error
    error = desired_position - theta;
    
    % Update PID controller
    integral = integral + error * dt;
    derivative = (error - previous_error) / dt;
    control_signal = Kp * error + Ki * integral + Kd * derivative;
    previous_error = error;
    
    % Saturate control signal (voltage) to a maximum value (e.g., 24V)
    V = min(max(control_signal, -24), 24);
    
    % Back EMF
    Eb = Kb * omega;
    
    % Differential equations
    dIa_dt = (V - R * Ia - Eb) / L;
    domega_dt = (Kt * Ia - B * omega) / J;
    
    % Euler method to update the state variables
    Ia = Ia + dIa_dt * dt;
    omega = omega + domega_dt * dt;
    theta = theta + omega * dt;
    
    % Store results
    Ia_arr(i) = Ia;
    omega_arr(i) = omega;
    theta_arr(i) = theta;
    error_arr(i) = error;
    control_signal_arr(i) = control_signal;
end

% Plot results
figure;
subplot(4,1,1);
plot(t, theta_arr / (2 * pi));
title('Number of Revolutions');
xlabel('Time (s)');
ylabel('Revolutions');

subplot(4,1,2);
plot(t, omega_arr);
title('Angular Velocity (rad/s)');
xlabel('Time (s)');
ylabel('omega (rad/s)');

subplot(4,1,3);
plot(t, Ia_arr);
title('Current (A)');
xlabel('Time (s)');
ylabel('Ia (A)');

subplot(4,1,4);
plot(t, control_signal_arr);
title('Control Signal (V)');
xlabel('Time (s)');
ylabel('Voltage (V)');
