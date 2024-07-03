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

% PI controller gains
Kp = 300;
Ki = 50;

% Desired RPM
desired_rpm = 600;  % Example value
desired_omega = (desired_rpm / 60) * 2 * pi;  % Convert RPM to rad/s

% Simulation parameters
t_final = 50;  % Simulation time (s)
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

% PI controller variables
integral = 0;
previous_error = 0;

% Simulation loop
for i = 1:length(t)
    % Calculate error
    error = desired_omega - omega;
    
    % Update PI controller
    integral = integral + error * dt;
    control_signal = Kp * error + Ki * integral;
    
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
plot(t, (omega_arr / (2 * pi)) * 60);
title('Motor RPM');
xlabel('Time (s)');
ylabel('RPM');
grid on;

subplot(4,1,2);
plot(t, omega_arr);
title('Angular Velocity (rad/s)');
xlabel('Time (s)');
ylabel('omega (rad/s)');
grid on;

subplot(4,1,3);
plot(t, Ia_arr);
title('Current (A)');
xlabel('Time (s)');
ylabel('Ia (A)');
grid on;

subplot(4,1,4);
plot(t, control_signal_arr);
title('Control Signal (V)');
xlabel('Time (s)');
ylabel('Voltage (V)');
grid on;

