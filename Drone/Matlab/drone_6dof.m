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

% PID gains for yaw, roll, and pitch
Kp_yaw = 2.5;  % Proportional gain for yaw
Ki_yaw = 0.5;  % Integral gain for yaw
Kd_yaw = 2.0;  % Derivative gain for yaw

Kp_pitch = 2.0;  % Proportional gain for pitch
Ki_pitch = 0.5;  % Integral gain for pitch
Kd_pitch = 2.0;  % Derivative gain for pitch

Kp_roll = 2.0;  % Proportional gain for roll
Ki_roll = 0.5;  % Integral gain for roll
Kd_roll = 2.0;  % Derivative gain for roll

% State-space matrices
A = [0, 0, 0, 1, 0, 0;   % x_dot = u
     0, 0, 0, 0, 1, 0;   % y_dot = v
     0, 0, 0, 0, 0, 1;   % z_dot = w
     0, 0, 0, 0, 0, 0;   % u_dot = controlled by input
     0, 0, 0, 0, 0, 0;   % v_dot = controlled by input
     0, 0, 0, 0, 0, 0];  % w_dot = controlled by input

B = [0, 0, 0; 
     0, 0, 0; 
     0, 0, 0; 
     1/m, 0, 0; 
     0, 1/m, 0; 
     0, 0, 1/m];

C = eye(6);
D = zeros(6, 3);

% Initial conditions
x0 = [0; 0; 0; 0; 0; 0];  % [x; y; z; u; v; w]
yaw0 = 0;
pitch0 = 0;
roll0 = 0;

% Updated waypoints for smoother yaw transitions, including return to origin
waypoints = [0, 0, 10; 
             5, 2, 12; 
             10, 5, 14; 
             15, 10, 16; 
             20, 15, 18; 
             25, 20, 20; 
             30, 25, 18; 
             35, 30, 16; 
             40, 35, 14; 
             45, 40, 12;
             0, 0, 10];  % Final waypoint returns to origin

current_waypoint = 1;
tolerance = 0.5; % Distance tolerance to consider waypoint reached

% Time step
dt = 0.01;
time = 0:dt:120;  % Extended time to accommodate return to origin

% Initialize state
x = x0;
yaw = yaw0;
pitch = pitch0;
roll = roll0;

% Initialize arrays for storing results
x_array = zeros(6, length(time));
yaw_array = zeros(1, length(time));
pitch_array = zeros(1, length(time));
roll_array = zeros(1, length(time));

% PID integrals
integral_error_pos = [0; 0; 0];
integral_error_yaw = 0;
integral_error_pitch = 0;
integral_error_roll = 0;

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

    % Update integral errors for position
    integral_error_pos = integral_error_pos + error_pos * dt;
    
    % Desired velocities (PID control)
    u_des = Kp_pos * error_pos(1) + Ki_pos * integral_error_pos(1) - Kd_pos * x(4);
    v_des = Kp_pos * error_pos(2) + Ki_pos * integral_error_pos(2) - Kd_pos * x(5);
    w_des = Kp_pos * error_pos(3) + Ki_pos * integral_error_pos(3) - Kd_pos * x(6);
    
    % Yaw control
    yaw_error = atan2(y_des - x(2), x_des - x(1)) - yaw;
    integral_error_yaw = integral_error_yaw + yaw_error * dt;
    yaw_dot = Kp_yaw * yaw_error + Ki_yaw * integral_error_yaw - Kd_yaw * yaw;

    % Pitch control
    pitch_error = u_des - pitch;  % Desired forward velocity vs. current pitch
    integral_error_pitch = integral_error_pitch + pitch_error * dt;
    pitch_dot = Kp_pitch * pitch_error + Ki_pitch * integral_error_pitch - Kd_pitch * pitch;

    % Roll control
    roll_error = v_des - roll;  % Desired lateral velocity vs. current roll
    integral_error_roll = integral_error_roll + roll_error * dt;
    roll_dot = Kp_roll * roll_error + Ki_roll * integral_error_roll - Kd_roll * roll;

    % Update roll, pitch, and yaw
    roll = roll + roll_dot * dt;
    pitch = pitch + pitch_dot * dt;
    yaw = yaw + yaw_dot * dt;

    % Store orientation for later analysis
    roll_array(t) = roll;
    pitch_array(t) = pitch;
    yaw_array(t) = yaw;
    
    % Input vector u(t) [u_des; v_des; w_des]
    u = [u_des; v_des; w_des];
    
    % State-space equation
    x_dot = A * x + B * u;
    
    % Update state variables
    x = x + x_dot * dt;
    
    % Store results
    x_array(:, t) = x;
end

% Plot results in 3D with directional arrows
figure;
plot3(x_array(1,:), x_array(2,:), x_array(3,:), 'b', 'LineWidth', 1.5);
hold on;
plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'ro', 'MarkerSize', 8, 'LineWidth', 2);

% Add direction arrows at intervals
for t = 1:100:length(time)
    % Calculate direction vector based on yaw, pitch, and roll
    direction = [cos(yaw_array(t)) * cos(pitch_array(t));
                 sin(yaw_array(t)) * cos(pitch_array(t));
                 sin(pitch_array(t))];
    % Normalize the direction vector
    direction = direction / norm(direction);
    
    % Plot the arrow
    quiver3(x_array(1,t), x_array(2,t), x_array(3,t), direction(1), direction(2), direction(3), 2, 'r', 'LineWidth', 1.2, 'MaxHeadSize', 1);
end

xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
title('Drone Path in 3D with Waypoints and Direction Arrows');
grid on;
hold off;

% Plot roll, pitch, yaw variations
figure;
subplot(3,1,1);
plot(time, roll_array, 'r');
xlabel('Time (s)');
ylabel('Roll (rad)');
title('Drone Roll Angle');

subplot(3,1,2);
plot(time, pitch_array, 'g');
xlabel('Time (s)');
ylabel('Pitch (rad)');
title('Drone Pitch Angle');

subplot(3,1,3);
plot(time, yaw_array, 'b');
xlabel('Time (s)');
ylabel('Yaw (rad)');
title('Drone Yaw Angle');
