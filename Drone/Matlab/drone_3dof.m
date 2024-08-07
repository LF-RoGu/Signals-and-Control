clc;
close all;
clear;

% Parameters
m = 1.0;    % mass of the drone (kg)
g = 9.81;   % gravitational acceleration (m/s^2)
Ix = 0.01;  % moment of inertia around x-axis (kg*m^2)
Iy = 0.01;  % moment of inertia around y-axis (kg*m^2)
Iz = 0.02;  % moment of inertia around z-axis (kg*m^2)
l = 0.1;    % distance from the center to the propeller (m)
b = 1e-6;   % thrust factor (N/(rad/s)^2)

% PID controller gains
Kp_pos = 1.0;  % Proportional gain for position
Kd_pos = 20.5;  % Derivative gain for position
Kp_yaw = 1.0;  % Proportional gain for yaw
Kd_yaw = 0.5;  % Derivative gain for yaw
Kp_pitch = 1.0; % Proportional gain for pitch
Kd_pitch = 1.5; % Derivative gain for pitch
Kp_roll = 1.0;  % Proportional gain for roll
Kd_roll = 1.5;  % Derivative gain for roll

% State variables
x = 0; y = 0; z = 10; % position (m), z is set to 10 for constant altitude
phi = 0; theta = 0; psi = 0; % orientation (rad)
u = 0; v = 0; w = 0; % linear velocities (m/s)
p = 0; q = 0; r = 0; % angular velocities (rad/s)

% Motor speeds (rad/s)
Omega1 = 1000;
Omega2 = 1000;
Omega3 = 1000;
Omega4 = 1000;

% Time step
dt = 0.01;
time = 0:dt:50;

% Initialize arrays for storing results
x_array = zeros(size(time));
y_array = zeros(size(time));
z_array = z * ones(size(time)); % constant altitude
phi_array = zeros(size(time));
theta_array = zeros(size(time));
psi_array = zeros(size(time));

% Custom path parameters
% Define the desired sine wave trajectory
desired_trajectory = @(t) [t, 5 * sin(0.5 * t)]; % sine wave along y-axis

% Simulation loop
for t = 1:length(time)
    % Desired position
    pos_des = desired_trajectory(time(t));
    x_des = pos_des(1);
    y_des = pos_des(2);
    
    % Compute desired velocities (finite difference approximation)
    if t == 1
        u_des = 0;
        v_des = 0;
    else
        u_des = (x_des - x_array(t-1)) / dt;
        v_des = (y_des - y_array(t-1)) / dt;
    end
    
    % Thrust and torques
    Ft = m * g; % constant thrust to counteract gravity
    tau_x = Kp_roll * (0 - phi) + Kd_roll * (0 - p);
    tau_y = Kp_pitch * (0 - theta) + Kd_pitch * (0 - q);
    tau_z = Kp_yaw * (0 - psi) + Kd_yaw * (0 - r);
    
    % Proportional control to follow the desired path
    Fx = Kp_pos * (x_des - x) + Kd_pos * (u_des - u);
    Fy = Kp_pos * (y_des - y) + Kd_pos * (v_des - v);
    
    % Update motor speeds based on control inputs
    Omega1 = sqrt((Ft / (4 * b)) - (tau_x / (2 * b * l)) - (tau_y / (2 * b * l)) - (tau_z / (4 * b * l)));
    Omega2 = sqrt((Ft / (4 * b)) - (tau_x / (2 * b * l)) + (tau_y / (2 * b * l)) + (tau_z / (4 * b * l)));
    Omega3 = sqrt((Ft / (4 * b)) + (tau_x / (2 * b * l)) + (tau_y / (2 * b * l)) - (tau_z / (4 * b * l)));
    Omega4 = sqrt((Ft / (4 * b)) + (tau_x / (2 * b * l)) - (tau_y / (2 * b * l)) + (tau_z / (4 * b * l)));
    
    % Ensure motor speeds are non-negative
    Omega1 = max(0, Omega1);
    Omega2 = max(0, Omega2);
    Omega3 = max(0, Omega3);
    Omega4 = max(0, Omega4);
    
    % Call the motor function to get the thrust for each motor
    thrust1 = dron_motor(time(t), Omega1 * (60 / (2 * pi))); % Convert rad/s to RPM
    thrust2 = dron_motor(time(t), Omega2 * (60 / (2 * pi)));
    thrust3 = dron_motor(time(t), Omega3 * (60 / (2 * pi)));
    thrust4 = dron_motor(time(t), Omega4 * (60 / (2 * pi)));
    
    % Total thrust and torques
    Ft = thrust1 + thrust2 + thrust3 + thrust4;
    tau_x = l * (thrust2 - thrust4);
    tau_y = l * (thrust3 - thrust1);
    tau_z = b * (Omega1 - Omega2 + Omega3 - Omega4);
    
    % Equations of motion
    x_dot = u;
    y_dot = v;
    z_dot = 0; % z_dot is zero to maintain constant altitude
    
    u_dot = r * v - q * w - g * theta + Fx / m;
    v_dot = p * w - r * u + g * phi + Fy / m;
    w_dot = 0; % w_dot is zero to maintain constant altitude
    
    p_dot = (Iy - Iz) * q * r / Ix + tau_x / Ix;
    q_dot = (Iz - Ix) * p * r / Iy + tau_y / Iy;
    r_dot = (Ix - Iy) * p * q / Iz + tau_z / Iz;
    
    phi_dot = p + q * sin(phi) * tan(theta) + r * cos(phi) * tan(theta);
    theta_dot = q * cos(phi) - r * sin(phi);
    psi_dot = q * sin(phi) / cos(theta) + r * cos(phi) / cos(theta);
    
    % Update state variables
    x = x + x_dot * dt;
    y = y + y_dot * dt;
    
    u = u + u_dot * dt;
    v = v + v_dot * dt;
    w = 0; % w is zero to maintain constant altitude
    
    p = p + p_dot * dt;
    q = q + q_dot * dt;
    r = r + r_dot * dt;
    
    phi = phi + phi_dot * dt;
    theta = theta + theta_dot * dt;
    psi = psi + psi_dot * dt;
    
    % Store results
    x_array(t) = x;
    y_array(t) = y;
    z_array(t) = z; % constant altitude
    phi_array(t) = phi;
    theta_array(t) = theta;
    psi_array(t) = psi;
end

% Plot results in 3D
figure;
plot3(x_array, y_array, z_array);
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
title('Drone Path in 3D');
grid on;

% Plot results in 2D
figure;
subplot(3,1,1);
plot(time, x_array);
xlabel('Time (s)');
ylabel('X Position (m)');
title('Drone X Position');

subplot(3,1,2);
plot(time, y_array);
xlabel('Time (s)');
ylabel('Y Position (m)');
title('Drone Y Position');

subplot(3,1,3);
plot(time, z_array);
xlabel('Time (s)');
ylabel('Z Position (m)');
title('Drone Z Position');

figure;
subplot(3,1,1);
plot(time, phi_array);
xlabel('Time (s)');
ylabel('Roll (rad)');
title('Drone Roll Angle');

subplot(3,1,2);
plot(time, theta_array);
xlabel('Time (s)');
ylabel('Pitch (rad)');
title('Drone Pitch Angle');

subplot(3,1,3);
plot(time, psi_array);
xlabel('Time (s)');
ylabel('Yaw (rad)');
title('Drone Yaw Angle');

figure;
plot(x_array, y_array);
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Drone Path');
