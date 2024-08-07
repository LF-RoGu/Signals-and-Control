close all;
clear;
clc;

% Time vector for state-space model
tspan = 0:0.01:50; % Using a finer time span to include multiple points
% Initial conditions
initial_conditions = [0, 0];
% Desired poles
desired_poles = [-4; -4];

% Pole for current sensor
p = 782590;

% Call the plotting function
dron_cdMotor(tspan, initial_conditions, desired_poles);

%% Functions
function dron_cdMotor(tspan, initial_conditions, desired_poles)
    global A B C D K F

    % Parameters for the motor model (used in the function scope)
    R = 15;
    L = 5 * 10^-3;
    Jm = 8.5 * 10^-6;
    B_friction = 3.5 * 10^-7;
    Kt = 12 * 10^-3;
    Kb = 12 * 10^-3;

    a = R / L;
    b = Kb / L;
    c = Kt / Jm;
    d = B_friction / Jm;
    e = 1 / L;

    % State-space model matrices
    A = [-a, -b; c, -d];
    B = [e; 0];
    C = [0, 1];
    D = 0;

    % Controllability matrix and gain calculations
    Mc = [B A * B];
    H = (A - desired_poles(1) * eye(2)) * (A - desired_poles(2) * eye(2));
    K = -[0 1] * inv(Mc) * H;
    F = 1 / (C * inv(-A - B * K) * B);

    % Solve the differential equations using ode45
    [t, X] = ode45(@(t, X) motor_cd_sys(t, X), tspan, initial_conditions);

    % Convert angular velocity to RPM
    rpms = X(:, 2) * (60 / (2 * pi));

    % Calculate thrust with realistic values
    thrust = calculate_thrust(rpms);

    % Plot results
    figure;
    subplot(3,1,1);
    plot(t, X(:,1));
    title('State 1: Current (i(t))');
    xlabel('Time (s)');
    ylabel('Current (A)');
    grid on;

    subplot(3,1,2);
    plot(t, rpms);
    title('State 2: Angular Velocity (RPM)');
    xlabel('Time (s)');
    ylabel('Angular Velocity (RPM)');
    grid on;

    subplot(3,1,3);
    plot(t, thrust);
    title('Thrust');
    xlabel('Time (s)');
    ylabel('Thrust (N)');
    grid on;
end

function dX = motor_cd_sys(t, X)
    global A B K F

    % Reference RPM function
    ref_rpm = ref_rpm_function(t);
    % Convert RPM to rad/s
    ref = ref_rpm * (2 * pi / 60);

    % Control input calculation
    U = K * X + F * ref;

    % State-space model differential equations
    dX = A * X + B * U;
end

function rpm = ref_rpm_function(t)
    % Reference RPM as a sine wave
    rpm = 2000 + 1000 * sin(0.1 * t);  % Example: sinusoidal reference RPM
end

function thrust = calculate_thrust(rpms)
    % Thrust calculation with realistic propeller values
    C_T = 0.1;      % Thrust coefficient
    rho = 1.225;    % Air density (kg/m³)
    D = 0.254;      % Propeller diameter (meters, approximately 10 inches)

    % Calculate thrust
    thrust = C_T * rho * (rpms / 60).^2 * D^4;
end
