clc;
close all;
clear;

% Parameters
m = 5.0;    % mass of the drone (kg)
g = 9.81;   % gravitational acceleration (m/s^2)
Ix = 0.01;  % Moment of inertia around x-axis (kg*m^2)
Iy = 0.01;  % Moment of inertia around y-axis (kg*m^2)
Iz = 0.02;  % Moment of inertia around z-axis (kg*m^2)

% PID controller gains
Kp_pos = 5.0;  % Proportional gain for position
Ki_pos = 0.4;  % Integral gain for position
Kd_pos = 6.8;  % Derivative gain for position

Kp_yaw = 4.0;  % Proportional gain for yaw
Ki_yaw = 0.3;  % Integral gain for yaw
Kd_yaw = 3.0;  % Derivative gain for yaw

Kp_pitch = 4.0;  % Proportional gain for pitch
Ki_pitch = 0.3;  % Integral gain for pitch
Kd_pitch = 3.0;  % Derivative gain for pitch

Kp_roll = 4.0;  % Proportional gain for roll
Ki_roll = 0.3;  % Integral gain for roll
Kd_roll = 3.0;  % Derivative gain for roll

% State-space matrices
A = [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0;   % x_dot = u
     0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0;   % y_dot = v
     0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;   % z_dot = w
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;   % u_dot = controlled by input
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;   % v_dot = controlled by input
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;   % w_dot = controlled by input
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0;   % phi_dot = p
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0;   % theta_dot = q
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;   % psi_dot = r
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;   % p_dot = controlled by input
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;   % q_dot = controlled by input
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];  % r_dot = controlled by input

B = [0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0;
     1/m, 0, 0, 0, 0, 0;
     0, 1/m, 0, 0, 0, 0;
     0, 0, 1/m, 0, 0, 0;
     0, 0, 0, 1/Ix, 0, 0;
     0, 0, 0, 0, 1/Iy, 0;
     0, 0, 0, 0, 0, 1/Iz;
     0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0];

C = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
     0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0];

D = [0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0];

% Initial conditions
x0 = [0; 0; 10; 0; 0; 0; 0; 0; 0; 0; 0; 0];  % [x; y; z; u; v; w; phi; theta; psi; p; q; r]

% Waypoints in 3D space
waypoints = [0, 0, 10; 10, 10, 15; 20, -5, 5; 5, 15, 12; -10, 5, 8];
current_waypoint = 1;
tolerance = 0.5; % Distance tolerance to consider waypoint reached

% Time step
dt = 0.01;
time = 0:dt:100;

% Initialize state
x = x0;

% Initialize arrays for storing results
x_array = zeros(12, length(time));

% PID integrals
integral_error_pos = [0; 0; 0];
integral_error_ori = [0; 0; 0];

% Simulation loop
for t = 1:length(time)
    % Check if the current waypoint is reached
    if norm([x(1) - waypoints(current_waypoint, 1), x(2) - waypoints(current_waypoint, 2), x(3) - waypoints(current_waypoint, 3)]) < tolerance
        % Move to the next waypoint if available
        if current_waypoint < size(waypoints, 1)
            current_waypoint = current_waypoint + 1;
        end
    end
    
    % Desired position and orientation
    x_des = waypoints(current_waypoint, 1);
    y_des = waypoints(current_waypoint, 2);
    z_des = waypoints(current_waypoint, 3);
    
    % Position errors
    error_pos = [x_des - x(1); y_des - x(2); z_des - x(3)];

    % Update integral errors
    integral_error_pos = integral_error_pos + error_pos * dt;
    
    % Derivative errors (for velocity control)
    vel_des = [Kp_pos * error_pos(1) + Ki_pos * integral_error_pos(1) - Kd_pos * x(4);
               Kp_pos * error_pos(2) + Ki_pos * integral_error_pos(2) - Kd_pos * x(5);
               Kp_pos * error_pos(3) + Ki_pos * integral_error_pos(3) - Kd_pos * x(6)];
    
    % Orientation control
    error_ori = [0 - x(7); 0 - x(8); 0 - x(9)];  % Assuming we want to keep level flight
    integral_error_ori = integral_error_ori + error_ori * dt;
    
    ang_vel_des = [Kp_roll * error_ori(1) + Ki_roll * integral_error_ori(1) - Kd_roll * x(10);
                   Kp_pitch * error_ori(2) + Ki_pitch * integral_error_ori(2) - Kd_pitch * x(11);
                   Kp_yaw * error_ori(3) + Ki_yaw * integral_error_ori(3) - Kd_yaw * x(12)];
    
    % Input vector u(t)
    u = [vel_des; ang_vel_des];
    
    % State-space equation
    x_dot = A * x + B * u;
    
    % Update state variables
    x = x + x_dot * dt;
    
    % Store results
    x_array(:, t) = x;
end

% Plot results in 3D
figure;
plot3(x_array(1,:), x_array(2,:), x_array(3,:), 'b', 'LineWidth', 1.5);
hold on;
plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
title('Drone Path in 3D with Waypoints');
grid on;
hold off;

% Plot results in 2D
figure;
subplot(3,1,1);
plot(time, x_array(1,:), 'b');
xlabel('Time (s)');
ylabel('X Position (m)');
title('Drone X Position');

subplot(3,1,2);
plot(time, x_array(2,:), 'b');
xlabel('Time (s)');
ylabel('Y Position (m)');
title('Drone Y Position');

subplot(3,1,3);
plot(time, x_array(3,:), 'b');
xlabel('Time (s)');
ylabel('Z Position (m)');
title('Drone Z Position');

figure;
plot(x_array(1,:), x_array(2,:), 'b', 'LineWidth', 1.5);
hold on;
plot(waypoints(:,1), waypoints(:,2), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Drone Path in 2D with Waypoints');
grid on;
hold off;
