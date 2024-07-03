% Parameters of the BLDC motor
L = 0.001;  % Inductance (H)
R = 1;      % Resistance (Ohms)
Kb = 0.01;  % Back EMF constant (V/(rad/s))
Kt = 0.01;  % Torque constant (Nm/A)
J = 0.01;   % Rotor inertia (kg*m^2)
B = 0.001;  % Damping coefficient (Nms)

% Simulation parameters
t_final = 10;  % Simulation time (s)
dt = 0.001;    % Time step (s)
t = 0:dt:t_final;

% Input (voltage applied to the motor)
V = 24 * ones(size(t));  % Constant voltage of 24V

% Initial conditions
Ia = 0;      % Initial current (A)
omega = 0;   % Initial angular velocity (rad/s)
theta = 0;   % Initial position (rad)

% Preallocate arrays for results
Ia_arr = zeros(size(t));
omega_arr = zeros(size(t));
theta_arr = zeros(size(t));

% Simulation loop
for i = 1:length(t)
    % Back EMF
    Eb = Kb * omega;
    
    % Differential equations
    dIa_dt = (V(i) - R * Ia - Eb) / L;
    domega_dt = (Kt * Ia - B * omega) / J;
    
    % Euler method to update the state variables
    Ia = Ia + dIa_dt * dt;
    omega = omega + domega_dt * dt;
    theta = theta + omega * dt;
    
    % Store results
    Ia_arr(i) = Ia;
    omega_arr(i) = omega;
    theta_arr(i) = theta;
end

% Plot results
figure;
subplot(3,1,1);
plot(t, Ia_arr);
title('Current (A)');
xlabel('Time (s)');
ylabel('Ia (A)');

subplot(3,1,2);
plot(t, omega_arr);
title('Angular Velocity (rad/s)');
xlabel('Time (s)');
ylabel('omega (rad/s)');

subplot(3,1,3);
plot(t, theta_arr);
title('Position (rad)');
xlabel('Time (s)');
ylabel('theta (rad)');
