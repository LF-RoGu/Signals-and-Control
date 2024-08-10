clc;
close all;
clear;

% Parameters
m = 5.0;    % mass of the drone (kg)
g = 9.81;   % gravitational acceleration (m/s^2)

% PID controller gains
Kp_pos = 5.0;  % Proportional gain for position
Ki_pos = 0.4;  % Integral gain for position
Kd_pos = 6.8;  % Derivative gain for position

% State-space matrices
A = [0, 0, 0, 1, 0, 0;
     0, 0, 0, 0, 1, 0;
     0, 0, 0, 0, 0, 1;
     0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0];

B = [0, 0, 0;
     0, 0, 0;
     0, 0, 0;
     1/m, 0, 0;
     0, 1/m, 0;
     0, 0, 1/m];

C = eye(6);  % Output matrix, assuming we want to observe all states
D = zeros(6, 3);  % No direct feedthrough

% Initial conditions
x0 = [0; 0; 10; 0; 0; 0];  % [x; y; z; u; v; w]

% Waypoints in the xy-plane
waypoints = [0, 0; 10, 10; 20, -5; 5, 15; -10, 5];
current_waypoint = 1;
tolerance = 0.5; % Distance tolerance to consider waypoint reached

% Time step
dt = 0.01;
time = 0:dt:50;

% Initialize state
x = x0;

% Initialize arrays for storing results
x_array = zeros(6, length(time));

% PID integrals
integral_error_x = 0;
integral_error_y = 0;
integral_error_z = 0;

% Simulation loop
for t = 1:length(time)
    % Check if the current waypoint is reached
    if norm([x(1) - waypoints(current_waypoint, 1), x(2) - waypoints(current_waypoint, 2)]) < tolerance
        % Move to the next waypoint if available
        if current_waypoint < size(waypoints, 1)
            current_waypoint = current_waypoint + 1;
        end
    end
    
    % Desired position
    x_des = waypoints(current_waypoint, 1);
    y_des = waypoints(current_waypoint, 2);
    z_des = x0(3);  % Maintain constant altitude
    
    % Position errors
    error_x = x_des - x(1);
    error_y = y_des - x(2);
    error_z = z_des - x(3);

    % Update integral errors
    integral_error_x = integral_error_x + error_x * dt;
    integral_error_y = integral_error_y + error_y * dt;
    integral_error_z = integral_error_z + error_z * dt;
    
    % Derivative errors (for velocity control)
    u_des = Kp_pos * error_x + Ki_pos * integral_error_x - Kd_pos * x(4);
    v_des = Kp_pos * error_y + Ki_pos * integral_error_y - Kd_pos * x(5);
    w_des = Kp_pos * error_z + Ki_pos * integral_error_z - Kd_pos * x(6);
    
    % Input vector u(t)
    u = [u_des; v_des; w_des];
    
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
plot3(waypoints(:,1), waypoints(:,2), x0(3) * ones(size(waypoints, 1)), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
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
