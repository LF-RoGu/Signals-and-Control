% Transfer function (a)
H1 = tf([1 0.1], [1]);

% Transfer function (b)
H2 = tf([1], [1 0.01]);

% Transfer function (c)
H3 = tf([1e4 1e4], [1 110 1000]);

% Bode plot for (a)
figure;
bode(H1);
title('Bode Plot of H(s) = 1 + s/10');
grid on;

% Bode plot for (b)
figure;
bode(H2);
title('Bode Plot of H(s) = 1 / (1 + s/100)');
grid on;

% Bode plot for (c)
figure;
bode(H3);
title('Bode Plot of H(s) = 10^4 (1 + s) / ((10 + s)(100 + s))');
grid on;
