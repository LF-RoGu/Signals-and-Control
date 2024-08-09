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
b = 1e-6;   % Thrust factor (N/(rad/s)^2)
d = 1e-7;   % Drag factor (N*m/(rad/s)^2)

% -------------------------------------------------------------------------
% PID Controller Gains
% -------------------------------------------------------------------------
Kp_pos = 8.0;  % Proportional gain for position
Ki_pos = 0.2;  % Integral gain for position (lowered to prevent windup)
Kd_pos = 28.0;  % Derivative gain for position (adjusted for stability)

Kp_yaw = 50.0;  % Proportional gain for yaw (yaw angle control)
Ki_yaw = 1.0;  % Integral gain for yaw
Kd_yaw = 12.0;  % Derivative gain for yaw

Kp_pitch = 50.0; % Proportional gain for pitch (pitch angle control)
Ki_pitch = 1.0; % Integral gain for pitch
Kd_pitch = 12.0; % Derivative gain for pitch

Kp_roll = 50.0;  % Proportional gain for roll (roll angle control)
Ki_roll = 5.0;  % Integral gain for roll
Kd_roll = 12.0;  % Derivative gain for roll

% -------------------------------------------------------------------------
% Initial State Variables
% -------------------------------------------------------------------------
% Start the drone at the origin [0, 0, 0]
x = 0; y = 0; z = 0; % Position (m)
phi = 0; theta = 0; psi = 0; % Orientation (rad)
u = 0; v = 0; w = 0; % Linear velocities (m/s)
p = 0; q = 0; r = 0; % Angular velocities (rad/s)

% -------------------------------------------------------------------------
% Waypoints (Target Positions)
% -------------------------------------------------------------------------
% Define waypoints in 3D space
waypoints = [10, 10, 12; 15, 5, 3; 20, 20, 10]; % [x1, y1, z1; x2, y2, z2; ...]

% -------------------------------------------------------------------------
% Motor Speeds Initialization
% -------------------------------------------------------------------------
% Initial motor speeds are set arbitrarily
Omega1 = 1000; % Motor 1 speed (rad/s)
Omega2 = 1000; % Motor 2 speed (rad/s)
Omega3 = 1000; % Motor 3 speed (rad/s)
Omega4 = 1000; % Motor 4 speed (rad/s)

% -------------------------------------------------------------------------
% Simulation Parameters
% -------------------------------------------------------------------------
dt = 0.01;  % Time step (s)
time = 0:dt:80;  % Simulation time vector (s)

% Arrays to store the drone's position, orientation, and motor speeds over time
x_array = zeros(size(time));
y_array = zeros(size(time));
z_array = zeros(size(time));
phi_array = zeros(size(time));
theta_array = zeros(size(time));
psi_array = zeros(size(time));
Omega1_array = zeros(size(time));
Omega2_array = zeros(size(time));
Omega3_array = zeros(size(time));
Omega4_array = zeros(size(time));

% -------------------------------------------------------------------------
% Control and Navigation Variables
% -------------------------------------------------------------------------
waypoint_idx = 1;  % Current waypoint index
tolerance = 0.3;   % Distance tolerance to consider waypoint reached (m)
max_velocity = 5;  % Maximum velocity (m/s) for smooth approach to waypoints

% Initialize PID integral errors
integral_error_x = 0;
integral_error_y = 0;
integral_error_z = 0;
integral_error_phi = 0;
integral_error_theta = 0;
integral_error_psi = 0;

% Anti-windup limits for integral errors
integral_limit = 13;  % Example limit, adjust as needed

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

    % Update integral errors with anti-windup
    integral_error_x = max(min(integral_error_x + error_x * dt, integral_limit), -integral_limit);
    integral_error_y = max(min(integral_error_y + error_y * dt, integral_limit), -integral_limit);
    integral_error_z = max(min(integral_error_z + error_z * dt, integral_limit), -integral_limit);
    
    % Calculate desired velocities with PID control
    u_des = Kp_pos * error_x + Ki_pos * integral_error_x - Kd_pos * u;
    v_des = Kp_pos * error_y + Ki_pos * integral_error_y - Kd_pos * v;
    w_des = Kp_pos * error_z + Ki_pos * integral_error_z - Kd_pos * w;
    
    % Calculate the distance to the target waypoint
    distance_to_target = norm([error_x, error_y, error_z]);
    
    % Limit the speed as the drone approaches the waypoint to prevent overshooting
    if distance_to_target < 5  % Start reducing speed within 10 meters of the waypoint
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
    
    % Update integral errors for orientation with anti-windup
    integral_error_phi = max(min(integral_error_phi + error_phi * dt, integral_limit), -integral_limit);
    integral_error_theta = max(min(integral_error_theta + error_theta * dt, integral_limit), -integral_limit);
    integral_error_psi = max(min(integral_error_psi + error_psi * dt, integral_limit), -integral_limit);

    % Calculate thrust and torques required to follow the desired path with PID control
    Ft = m * (g + w_des); % Thrust needed to counteract gravity and control altitude
    Ft = max(0, min(Ft, 20)); % Cap the thrust to a reasonable value

    tau_x = Kp_roll * error_phi + Ki_roll * integral_error_phi - Kd_roll * p;  % Roll torque
    tau_y = Kp_pitch * error_theta + Ki_pitch * integral_error_theta - Kd_pitch * q;  % Pitch torque
    tau_z = Kp_yaw * error_psi + Ki_yaw * integral_error_psi - Kd_yaw * r;  % Yaw torque
    
    % Calculate forces in the x, y, and z directions
    Fx = m * u_des;
    Fy = m * v_des;
    Fz = Ft - m * g; % Net force in the z direction
    
    % Update motor speeds based on control inputs
    Omega1 = sqrt((Ft / (4 * b)) - (tau_x / (2 * b * l)) - (tau_y / (2 * b * l)) - (tau_z / (4 * d)));
    Omega2 = sqrt((Ft / (4 * b)) - (tau_x / (2 * b * l)) + (tau_y / (2 * b * l)) + (tau_z / (4 * d)));
    Omega3 = sqrt((Ft / (4 * b)) + (tau_x / (2 * b * l)) + (tau_y / (2 * b * l)) - (tau_z / (4 * d)));
    Omega4 = sqrt((Ft / (4 * b)) + (tau_x / (2 * b * l)) - (tau_y / (2 * b * l)) + (tau_z / (4 * d)));
    
    % Ensure motor speeds are non-negative (physical constraint)
    Omega1 = max(0, min(Omega1, 2000)); % Example motor speed limit
    Omega2 = max(0, min(Omega2, 2000));
    Omega3 = max(0, min(Omega3, 2000));
    Omega4 = max(0, min(Omega4, 2000));
    
    % Store motor speeds for visualization
    Omega1_array(t) = Omega1;
    Omega2_array(t) = Omega2;
    Omega3_array(t) = Omega3;
    Omega4_array(t) = Omega4;
    
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
    w_dot = q * u - p * v + Fz / m;  % Change in z-velocity
    
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
% The following sections plot the drone's path, orientation, and motor
% speeds over time, along with markers showing the locations of the waypoints.

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

% Plot motor speeds over time
figure;
subplot(4,1,1);
plot(time, Omega1_array);
xlabel('Time (s)');
ylabel('Motor 1 Speed (rad/s)');
title('Motor 1 Speed');

subplot(4,1,2);
plot(time, Omega2_array);
xlabel('Time (s)');
ylabel('Motor 2 Speed (rad/s)');
title('Motor 2 Speed');

subplot(4,1,3);
plot(time, Omega3_array);
xlabel('Time (s)');
ylabel('Motor 3 Speed (rad/s)');
title('Motor 3 Speed');

subplot(4,1,4);
plot(time, Omega4_array);
xlabel('Time (s)');
ylabel('Motor 4 Speed (rad/s)');
title('Motor 4 Speed');
