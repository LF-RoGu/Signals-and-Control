function thrust = bldc_motor(t, v_ref)
    % Parameters for the BLDC motor model
    R = 0.05;            % Resistance (Ohms)
    L = 0.001;           % Inductance (Henries)
    J = 0.00001;         % Moment of inertia (kg*m^2)
    B_friction = 0.0001; % Friction coefficient (N*m*s)
    Kt = 0.01;           % Torque constant (N*m/A)
    Ke = 0.01;           % Back-EMF constant (V*s/rad)
    
    % State-space model matrices
    A = [-R/L, -Ke/L; Kt/J, -B_friction/J];
    B = [1/L; 0];
    C = [0, 1];
    D = 0;
    
    % Desired poles for the system
    desired_poles = [-50; -50];
    
    % Controllability matrix and gain calculations
    Mc = [B A * B];
    H = (A - desired_poles(1) * eye(2)) * (A - desired_poles(2) * eye(2));
    K = -[0 1] * inv(Mc) * H;
    F = 1 / (C * inv(-A - B * K) * B);
    
    % Initial conditions
    initial_conditions = [0, 0];
    
    % Solve the differential equations using ode45
    [~, X] = ode45(@(t, X) motor_cd_sys(t, X, v_ref, A, B, K, F), [0 0.1], initial_conditions);
    
    % Get the angular velocity (rad/s)
    angular_velocity = X(end, 2);
    rpm = angular_velocity * (60 / (2 * pi));
    
    % Thrust calculation with realistic propeller values
    thrust = calculate_thrust(rpm);
end

function dX = motor_cd_sys(t, X, v_ref, A, B, K, F)
    % Control input calculation
    U = K * X + F * v_ref;
    
    % State-space model differential equations
    dX = A * X + B * U;
end

function thrust = calculate_thrust(rpms)
    % Thrust calculation with realistic propeller values
    C_T = 0.2;      % Thrust coefficient
    rho = 1.225;    % Air density (kg/m³)
    D = 0.5;        % Propeller diameter (meters, approximately 20 inches)

    % Calculate thrust
    thrust = C_T * rho * (rpms / 60).^2 * D^4;
end
