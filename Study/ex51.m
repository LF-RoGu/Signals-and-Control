% Define the transfer functions
G1 = tf([1], [1 2 3]);
G2 = tf([1], [1 0.25]);
G3 = tf([1], [1 -0.25]);
G4 = tf([1], [1 2.25 3.5 0.75]);

% Define frequency range for Bode plot
omega = logspace(-2, 2, 1000); % from 0.01 to 100 rad/s

% Bode plot for G1
figure;
bode(G1, omega);
title('Bode Plot of G(s) = 1 / (s^2 + 2s + 3)');
grid on;

% Bode plot for G2
figure;
bode(G2, omega);
title('Bode Plot of G(s) = 1 / (s + 0.25)');
grid on;

% Bode plot for G3
figure;
bode(G3, omega);
title('Bode Plot of G(s) = 1 / (s - 0.25)');
grid on;

% Bode plot for G4
figure;
bode(G4, omega);
title('Bode Plot of G(s) = 1 / (s^3 + 2.25s^2 + 3.5s + 0.75)');
grid on;

% Define closed-loop transfer functions with unity feedback
G1_cl = feedback(G1, 1);
G2_cl = feedback(G2, 1);
G3_cl = feedback(G3, 1);
G4_cl = feedback(G4, 1);

%% Script to print the poles of the transfer functions
disp('Poles of the transfer functions:');
disp('---------------------------------');
disp('G1 (open-loop):');
disp(pole(G1));

disp('G1 (closed-loop):');
disp(pole(G1_cl));

disp('G2 (open-loop):');
disp(pole(G2));

disp('G2 (closed-loop):');
disp(pole(G2_cl));

disp('G3 (open-loop):');
disp(pole(G3));

disp('G3 (closed-loop):');
disp(pole(G3_cl));

disp('G4 (open-loop):');
disp(pole(G4));

disp('G4 (closed-loop):');
disp(pole(G4_cl));

%% Script to plot the poles of the transfer functions
% Unit step response for G1 (open-loop)
figure;
step(G1);
title('Unit Step Response of Open-Loop G(s) = 1 / (s^2 + 2s + 3)');
grid on;

% Unit step response for G1_cl (closed-loop)
figure;
step(G1_cl);
title('Unit Step Response of Closed-Loop G(s) = 1 / (s^2 + 2s + 3)');
grid on;

% Unit step response for G2 (open-loop)
figure;
step(G2);
title('Unit Step Response of Open-Loop G(s) = 1 / (s + 0.25)');
grid on;

% Unit step response for G2_cl (closed-loop)
figure;
step(G2_cl);
title('Unit Step Response of Closed-Loop G(s) = 1 / (s + 0.25)');
grid on;

% Unit step response for G3 (open-loop)
figure;
step(G3);
title('Unit Step Response of Open-Loop G(s) = 1 / (s - 0.25)');
grid on;

% Unit step response for G3_cl (closed-loop)
figure;
step(G3_cl);
title('Unit Step Response of Closed-Loop G(s) = 1 / (s - 0.25)');
grid on;

% Unit step response for G4 (open-loop)
figure;
step(G4);
title('Unit Step Response of Open-Loop G(s) = 1 / (s^3 + 2.25s^2 + 3.5s + 0.75)');
grid on;

% Unit step response for G4_cl (closed-loop)
figure;
step(G4_cl);
title('Unit Step Response of Closed-Loop G(s) = 1 / (s^3 + 2.25s^2 + 3.5s + 0.75)');
grid on;
