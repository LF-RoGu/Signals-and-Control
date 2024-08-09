clc;
close all;
clear;

% -------------------------------------------------------------------------
% Drone Physical Parameters
% -------------------------------------------------------------------------
m = 1.0;    % Mass of the drone (kg)
g = 9.81;   % Gravitational acceleration (m/s^2)
Ix = 0.01;  % Moment of inertia around x-axis (kg*m^2)
Iy = 0.01;  % Moment of inertia around y-axis (kg*m^2)
Iz = 0.02;  % Moment of inertia around z-axis (kg*m^2)
l = 0.1;    % Distance from the center to the propeller (m)
b = 1e-6;   % Thrust factor (N/(rad/s)^2) - how much thrust is generated per rad/s
d = 1e-7;   % Drag factor (N*m/(rad/s)^2) - how much drag torque is generated per rad/s

% -------------------------------------------------------------------------
% PID Controller Gains
% -------------------------------------------------------------------------
Kp_pos = 10.0;  % Proportional gain for position
Ki_pos = 2.0;  % Integral gain for position
Kd_pos = 10.5;  % Derivative gain for position

Kp_yaw = 5.0;  % Proportional gain for yaw (yaw angle control)
Ki_yaw = 2.5;  % Integral gain for yaw
Kd_yaw = 5.5;  % Derivative gain for yaw

Kp_pitch = 5.0; % Proportional gain for pitch (pitch angle control)
Ki_pitch = 2.5; % Integral gain for pitch
Kd_pitch = 5.5; % Derivative gain for pitch

Kp_roll = 2.0;  % Proportional gain for roll (roll angle control)
Ki_roll = 2.5;  % Integral gain for roll
Kd_roll = 5.5;  % Derivative gain for roll

% -------------------------------------------------------------------------
% Initial State Variables
% -------------------------------------------------------------------------
x = 0; y = 0; z = 10; % Position (m)
phi = 0; theta = 0; psi = 0; % Orientation (rad)
u = 0; v = 0; w = 0; % Linear velocities (m/s)
p = 0; q = 0; r = 0; % Angular velocities (rad/s)

% -------------------------------------------------------------------------
% Waypoints (Target Positions)
% -------------------------------------------------------------------------
waypoints = [10, 10, 12; 15, 5, 8; 20, 20, 10]; % [x1, y1, z1; x2, y2, z2; ...]

% -------------------------------------------------------------------------
% Motor Speeds Initialization
% -------------------------------------------------------------------------
Omega1 = 1000; % Motor 1 speed (rad/s)
Omega2 = 1000; % Motor 2 speed (rad/s)
Omega3 = 1000; % Motor 3 speed (rad/s)
Omega4 = 1000; % Motor 4 speed (rad/s)

% -------------------------------------------------------------------------
% Simulation Parameters
% -------------------------------------------------------------------------
dt = 0.01;  % Time step (s)
time = 0:dt:50;  % Simulation time vector (s)

% Arrays to store the drone's position and orientation over time
x_array = zeros(size(time));
y_array = zeros(size(time));
z_array = zeros(size(time));
phi_array = zeros(size(time));
theta_array = zeros(size(time));
psi_array = zeros(size(time));

% -------------------------------------------------------------------------
% Control and Navigation Variables
% -------------------------------------------------------------------------
waypoint_idx = 1;  % Current waypoint index
tolerance = 0.1;   % Distance tolerance to consider waypoint reached (m)
max_velocity = 2;  % Maximum velocity (m/s) for smooth approach to waypoints

% Initialize PID integral errors
integral_error_x = 0;
integral_error_y = 0;
integral_error_z = 0;
integral_error_phi = 0;
integral_error_theta = 0;
integral_error_psi = 0;

% -------------------------------------------------------------------------
% Simulation Loop
% -------------------------------------------------------------------------
for t = 1:length(time)
    % Determine the current target waypoint
    if waypoint_idx <= size(waypoints, 1)
        x_target = waypoints(waypoint_idx, 1);
        y_target = waypoints(waypoint_idx, 2);
        z_target = waypoints(waypoint_idx, 3);
    else
        x_target = waypoints(end, 1);
        y_target = waypoints(end, 2);
        z_target = waypoints(end, 3);
    end
    
    % Desired position based on the current waypoint
    x_des = x_target;
    y_des = y_target;
    z_des = z_target;
    
    % Calculate position errors
    error_x = x_des - x;
    error_y = y_des - y;
    error_z = z_des - z;

    % Update integral errors
    integral_error_x = integral_error_x + error_x * dt;
    integral_error_y = integral_error_y + error_y * dt;
    integral_error_z = integral_error_z + error_z * dt;
    
    % Calculate desired velocities with PID control
    u_des = Kp_pos * error_x + Ki_pos * integral_error_x - Kd_pos * u;
    v_des = Kp_pos * error_y + Ki_pos * integral_error_y - Kd_pos * v;
    w_des = Kp_pos * error_z + Ki_pos * integral_error_z - Kd_pos * w;
    
    % Calculate the distance to the target waypoint
    distance_to_target = norm([error_x, error_y, error_z]);
    
    % Limit the speed as the drone approaches the waypoint to prevent overshooting
    if distance_to_target < 5  % Start reducing speed within 5 meters of the waypoint
        scale_factor = distance_to_target / 5;
        u_des = max_velocity * scale_factor * sign(u_des);
        v_des = max_velocity * scale_factor * sign(v_des);
        w_des = max_velocity * scale_factor * sign(w_des);
    else
        u_des = max_velocity * sign(u_des);
        v_des = max_velocity * sign(v_des);
        w_des = max_velocity * sign(w_des);
    end
    
    % Orientation control errors
    error_phi = 0 - phi;
    error_theta = 0 - theta;
    error_psi = 0 - psi;
    
    % Update integral errors for orientation
    integral_error_phi = integral_error_phi + error_phi * dt;
    integral_error_theta = integral_error_theta + error_theta * dt;
    integral_error_psi = integral_error_psi + error_psi * dt;

    % Calculate thrust and torques required to follow the desired path with PID control
    Ft = m * (g + w_des); % Thrust needed to counteract gravity and control altitude
    tau_x = Kp_roll * error_phi + Ki_roll * integral_error_phi - Kd_roll * p;  % Roll torque
    tau_y = Kp_pitch * error_theta + Ki_pitch * integral_error_theta - Kd_pitch * q;  % Pitch torque
    tau_z = Kp_yaw * error_psi + Ki_yaw * integral_error_psi - Kd_yaw * r;  % Yaw torque
    
    % Proportional control to follow the desired trajectory
    Fx = Kp_pos * error_x + Ki_pos * integral_error_x - Kd_pos * u;  % Force in x-direction
    Fy = Kp_pos * error_y + Ki_pos * integral_error_y - Kd_pos * v;  % Force in y-direction
    
    % Update motor speeds based on control inputs
    Omega1 = sqrt((Ft / (4 * b)) - (tau_x / (2 * b * l)) - (tau_y / (2 * b * l)) - (tau_z / (4 * d)));
    Omega2 = sqrt((Ft / (4 * b)) - (tau_x / (2 * b * l)) + (tau_y / (2 * b * l)) + (tau_z / (4 * d)));
    Omega3 = sqrt((Ft / (4 * b)) + (tau_x / (2 * b * l)) + (tau_y / (2 * b * l)) - (tau_z / (4 * d)));
    Omega4 = sqrt((Ft / (4 * b)) + (tau_x / (2 * b * l)) - (tau_y / (2 * b * l)) + (tau_z / (4 * d)));
    
    % Ensure motor speeds are non-negative (physical constraint)
    Omega1 = max(0, Omega1);
    Omega2 = max(0, Omega2);
    Omega3 = max(0, Omega3);
    Omega4 = max(0, Omega4);
    
    % ---------------------------------------------------------------------
    % Equations of Motion
    % ---------------------------------------------------------------------
    % These equations update the drone's position and orientation based on
    % the forces and torques calculated by the PID controllers.
    x_dot = u;  % Change in x-position
    y_dot = v;  % Change in y-position
    z_dot = w;  % Change in z-position
    
    u_dot = r * v - q * w + Fx / m;  % Change in x-velocity
    v_dot = p * w - r * u + Fy / m;  % Change in y-velocity
    w_dot = q * u - p * v + Ft / m - g;  % Change in z-velocity
    
    p_dot = (Iy - Iz) * q * r / Ix + tau_x / Ix;  % Change in roll rate
    q_dot = (Iz - Ix) * p * r / Iy + tau_y / Iy;  % Change in pitch rate
    r_dot = (Ix - Iy) * p * q / Iz + tau_z / Iz;  % Change in yaw rate
    
    phi_dot = p + q * sin(phi) * tan(theta) + r * cos(phi) * tan(theta);  % Change in roll angle
    theta_dot = q * cos(phi) - r * sin(phi);  % Change in pitch angle
    psi_dot = q * sin(phi) / cos(theta) + r * cos(phi) / cos(theta);  % Change in yaw angle
    
    % ---------------------------------------------------------------------
    % Update State Variables
    % ---------------------------------------------------------------------
    % The state variables (position, velocity, orientation) are updated
    % based on the calculated derivatives.
    x = x + x_dot * dt;
    y = y + y_dot * dt;
    z = z + z_dot * dt;
    
    u = u + u_dot * dt;
    v = v + v_dot * dt;
    w = w + w_dot * dt;
    
    p = p + p_dot * dt;
    q = q + q_dot * dt;
    r = r + r_dot * dt;
    
    phi = phi + phi_dot * dt;
    theta = theta + theta_dot * dt;
    psi = psi + psi_dot * dt;
    
    % ---------------------------------------------------------------------
    % Store Results for Visualization
    % ---------------------------------------------------------------------
    % The drone's position and orientation are stored at each time step to
    % be plotted later.
    x_array(t) = x;
    y_array(t) = y;
    z_array(t) = z;
    phi_array(t) = phi;
    theta_array(t) = theta;
    psi_array(t) = psi;

    % ---------------------------------------------------------------------
    % Waypoint Navigation
    % ---------------------------------------------------------------------
    % If the drone is within the specified tolerance of the current waypoint,
    % move to the next waypoint in the sequence.
    if norm([x - x_target, y - y_target, z - z_target]) < tolerance
        waypoint_idx = waypoint_idx + 1; % Move to the next waypoint
    end
end

% -------------------------------------------------------------------------
% Visualization
% -------------------------------------------------------------------------
% The following sections plot the drone's path and orientation over time,
% along with markers showing the locations of the waypoints.

% Plot results in 3D with waypoints
figure;
plot3(x_array, y_array, z_array, 'b');  % Drone path in blue
hold on;
plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'rs', 'MarkerSize', 10, 'LineWidth', 2);  % Waypoints as red squares
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

% 2D Plot of drone path with waypoints
figure;
plot3(x_array, y_array, z_array, 'b');  % Drone path in blue
hold on;
plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'rs', 'MarkerSize', 10, 'LineWidth', 2);  % Waypoints as red squares
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
title('Drone Path with Waypoints');
grid on;
hold off;

% Plot orientation angles over time
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
