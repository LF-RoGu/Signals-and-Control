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
d = 1e-7;   % drag factor (N*m/(rad/s)^2)

% PID controller gains
Kp_pos = 5.0;  % Proportional gain for position (increased for better control)
Kd_pos = 1.0;  % Derivative gain for position (increased for damping)
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

% Waypoints
waypoints = [10, 10; 15, 5; 20, 20]; % [x1, y1; x2, y2; ...]

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

% Current waypoint index
waypoint_idx = 1;
tolerance = 0.1; % reduced tolerance for more precise waypoint achievement
max_velocity = 2; % maximum velocity (m/s) for smooth approach

% Simulation loop
for t = 1:length(time)
    % Check if the current waypoint is reached
    if waypoint_idx <= size(waypoints, 1)
        x_target = waypoints(waypoint_idx, 1);
        y_target = waypoints(waypoint_idx, 2);
    else
        x_target = waypoints(end, 1);
        y_target = waypoints(end, 2);
    end
    
    % Desired position based on target coordinates
    x_des = x_target;
    y_des = y_target;
    
    % Compute desired velocities (finite difference approximation)
    if t == 1
        u_des = 0;
        v_des = 0;
    else
        u_des = (x_des - x_array(t-1)) / dt;
        v_des = (y_des - y_array(t-1)) / dt;
    end
    
    % Calculate distance to target
    distance_to_target = norm([x_des - x, y_des - y]);
    
    % Velocity control: limit speed as drone approaches waypoint
    if distance_to_target < 5  % Start reducing speed within 5 meters of the waypoint
        scale_factor = distance_to_target / 5;
        u_des = max_velocity * scale_factor * sign(u_des);
        v_des = max_velocity * scale_factor * sign(v_des);
    else
        u_des = max_velocity * sign(u_des);
        v_des = max_velocity * sign(v_des);
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
    Omega1 = sqrt((Ft / (4 * b)) - (tau_x / (2 * b * l)) - (tau_y / (2 * b * l)) - (tau_z / (4 * d)));
    Omega2 = sqrt((Ft / (4 * b)) - (tau_x / (2 * b * l)) + (tau_y / (2 * b * l)) + (tau_z / (4 * d)));
    Omega3 = sqrt((Ft / (4 * b)) + (tau_x / (2 * b * l)) + (tau_y / (2 * b * l)) - (tau_z / (4 * d)));
    Omega4 = sqrt((Ft / (4 * b)) + (tau_x / (2 * b * l)) - (tau_y / (2 * b * l)) + (tau_z / (4 * d)));
    
    % Ensure motor speeds are non-negative
    Omega1 = max(0, Omega1);
    Omega2 = max(0, Omega2);
    Omega3 = max(0, Omega3);
    Omega4 = max(0, Omega4);
    
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

    % Check if the drone is within the tolerance of the current waypoint
    if norm([x - x_target, y - y_target]) < tolerance
        waypoint_idx = waypoint_idx + 1; % Move to the next waypoint
    end
end

% Plot results in 3D with waypoints
figure;
plot3(x_array, y_array, z_array, 'b');
hold on;
plot3(waypoints(:,1), waypoints(:,2), z * ones(size(waypoints, 1), 1), 'rs', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
title('Drone Path in 3D with Waypoints');
grid on;
hold off;

% Plot results in 2D with waypoints
figure;
subplot(3,1,1);
plot(time, x_array, 'b');
xlabel('Time (s)');
ylabel('X Position (m)');
title('Drone X Position');

subplot(3,1,2);
plot(time, y_array, 'b');
xlabel('Time (s)');
ylabel('Y Position (m)');
title('Drone Y Position');

subplot(3,1,3);
plot(time, z_array, 'b');
xlabel('Time (s)');
ylabel('Z Position (m)');
title('Drone Z Position');

figure;
plot(x_array, y_array, 'b');
hold on;
plot(waypoints(:,1), waypoints(:,2), 'rs', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Drone Path with Waypoints');
grid on;
hold off;

% Plot orientation angles
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
