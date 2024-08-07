% Parameters
m = 1.0;    % mass of the drone (kg)
g = 9.81;   % gravitational acceleration (m/s^2)
Ix = 0.01;  % moment of inertia around x-axis (kg*m^2)
Iy = 0.01;  % moment of inertia around y-axis (kg*m^2)
Iz = 0.02;  % moment of inertia around z-axis (kg*m^2)
l = 0.1;    % distance from the center to the propeller (m)
b = 1e-6;   % thrust factor (N/(rad/s)^2)
d = 1e-7;   % drag factor (N*m/(rad/s)^2)

% State variables
x = 0; y = 0; z = 0; % position (m)
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
time = 0:dt:10;

% Initialize arrays for storing results
x_array = zeros(size(time));
y_array = zeros(size(time));
psi_array = zeros(size(time));

% Simulation loop
for t = 1:length(time)
    % Thrust and torques
    Ft = b * (Omega1^2 + Omega2^2 + Omega3^2 + Omega4^2);
    tau_x = b * l * (Omega4^2 - Omega2^2);
    tau_y = b * l * (Omega3^2 - Omega1^2);
    tau_z = d * (Omega2^2 + Omega4^2 - Omega1^2 - Omega3^2);
    
    % Equations of motion
    x_dot = u;
    y_dot = v;
    z_dot = w;
    
    u_dot = r*v - q*w - g*theta;
    v_dot = p*w - r*u + g*phi;
    w_dot = q*u - p*v + Ft/m;
    
    p_dot = (Iy - Iz) * q * r / Ix + tau_x / Ix;
    q_dot = (Iz - Ix) * p * r / Iy + tau_y / Iy;
    r_dot = (Ix - Iy) * p * q / Iz + tau_z / Iz;
    
    phi_dot = p + q * sin(phi) * tan(theta) + r * cos(phi) * tan(theta);
    theta_dot = q * cos(phi) - r * sin(phi);
    psi_dot = q * sin(phi) / cos(theta) + r * cos(phi) / cos(theta);
    
    % Update state variables
    x = x + x_dot * dt;
    y = y + y_dot * dt;
    psi = psi + psi_dot * dt;
    
    u = u + u_dot * dt;
    v = v + v_dot * dt;
    w = w + w_dot * dt;
    
    p = p + p_dot * dt;
    q = q + q_dot * dt;
    r = r + r_dot * dt;
    
    phi = phi + phi_dot * dt;
    theta = theta + theta_dot * dt;
    
    % Store results
    x_array(t) = x;
    y_array(t) = y;
    psi_array(t) = psi;
end

% Plot results
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
plot(time, psi_array);
xlabel('Time (s)');
ylabel('Yaw (rad)');
title('Drone Yaw Angle');
