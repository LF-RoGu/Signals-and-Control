function thrust = dron_motor(t, w_ref)
    % Parameters for the motor model
    R = 15;
    L = 5 * 10^-3;
    Jm = 8.5 * 10^-6;
    B_friction = 3.5 * 10^-7;
    Kt = 12 * 10^-3;
    Kb = 12 * 10^-3;

    % State-space model matrices
    a = R / L;
    b = Kb / L;
    c = Kt / Jm;
    d = B_friction / Jm;
    e = 1 / L;
    
    A = [-a, -b; c, -d];
    B = [e; 0];
    C = [0, 1];
    D = 0;

    % Desired poles
    desired_poles = [-4; -4];
    
    % Controllability matrix and gain calculations
    Mc = [B A * B];
    H = (A - desired_poles(1) * eye(2)) * (A - desired_poles(2) * eye(2));
    K = -[0 1] * inv(Mc) * H;
    F = 1 / (C * inv(-A - B * K) * B);
    
    % Initial conditions
    initial_conditions = [0, 0];
    
    % Solve the differential equations using ode45
    [~, X] = ode45(@(t, X) motor_cd_sys(t, X, w_ref, A, B, K, F), [0 0.01], initial_conditions);
    
    % Get the angular velocity (RPM)
    angular_velocity = X(end, 2);
    rpm = angular_velocity * (60 / (2 * pi));
    
    % Thrust calculation with realistic propeller values
    thrust = calculate_thrust(rpm);
end

function dX = motor_cd_sys(t, X, w_ref, A, B, K, F)
    % Reference angular velocity
    ref = w_ref * (2 * pi / 60);
    
    % Control input calculation
    U = K * X + F * ref;
    
    % State-space model differential equations
    dX = A * X + B * U;
end

function thrust = calculate_thrust(rpms)
    % Thrust calculation with realistic propeller values
    C_T = 0.1;      % Thrust coefficient
    rho = 1.225;    % Air density (kg/m³)
    D = 0.254;      % Propeller diameter (meters, approximately 10 inches)

    % Calculate thrust
    thrust = C_T * rho * (rpms / 60).^2 * D^4;
end
