% Modelo identificado del motor DC
% Generado automáticamente

%% Parámetros del modelo continuo
K = 999.3659;      % Ganancia [RPM/V]
tau = 0.3008;  % Constante de tiempo [s]
Ts = 0.1887;              % Período de muestreo [s]

%% Crear función de transferencia continua
num_cont = K;
den_cont = [tau 1];
sys_cont = tf(num_cont, den_cont);

disp('Modelo continuo:');
disp(sys_cont);

%% Discretizar usando Tustin (bilinear)
sys_disc = c2d(sys_cont, Ts, 'tustin');

disp('Modelo discreto:');
disp(sys_disc);

%% Visualizar respuesta al escalón
figure('Name', 'Respuesta al Escalón');
step(sys_cont, sys_disc);
legend('Continuo', 'Discreto (Tustin)');
grid on;

%% Diagrama de Bode
figure('Name', 'Diagrama de Bode');
bode(sys_cont);
grid on;

%% Información adicional
info.K = K;
info.tau = tau;
info.Ts = Ts;
info.polo = -1/tau;
info.BW_hz = 0.5301;
info.ajuste = 99.97;

disp(' ');
disp('Información del modelo:');
disp(info);
