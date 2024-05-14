% Sistem Parametreleri
m = 1;  % Kütle (kg)
k = 37.20;  % Yay sabiti (N/m)
c = 2.24;  % Sönüm katsayısı (Ns/m)

% Sistemin Transfer Fonksiyonu
numerator = 1;
denominator = [m, c, k];
sys_tf = tf(numerator, denominator);

% Adım Yanıtı ve Performans Metriği Hesaplama
figure;
step(sys_tf);
title('Sistemin Adım Yanıtı');
grid on;

% Performans Metriği Hesaplama
info = stepinfo(sys_tf);

% Performans Metriği Değerleri
settling_time = info.SettlingTime;
overshoot = info.Overshoot;
peak_time = info.PeakTime;
steady_state_value = info.SettlingMin;  % Yerleşme değerini kabul ederek

% Hesaplanan Değerlerin Gösterimi
disp('Yerleşme Süresi: '), disp(settling_time);
disp('Aşma: '), disp(overshoot);
disp('Tepe Zamanı: '), disp(peak_time);
disp('Durulma Değeri: '), disp(steady_state_value);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

% MATLAB Code for System Characterization and PID Controller Design

% Given Parameters
m = 1;  % Mass (kg)
k = 37.20;  % Spring constant (N/m)
c = 2.24;  % Damping coefficient (Ns/m)

% Transfer Function of the System
numerator = 1;
denominator = [m, c, k];
sys_tf = tf(numerator, denominator);

% Plot Step Response
figure;
step(sys_tf);
title('Step Response of the System');
grid on;

% Calculate Performance Metrics
info = stepinfo(sys_tf);

% Display Performance Metrics
disp('Settling Time: '), disp(info.SettlingTime);
disp('Overshoot: '), disp(info.Overshoot);
disp('Peak Time: '), disp(info.PeakTime);
disp('Steady State Value: '), disp(info.SettlingMin);  % Assuming settling value

% PID Controller Design
Kp = 350;  % Proportional Gain
Ki = 300;  % Integral Gain
Kd = 50;   % Derivative Gain

% PID Controller
pid_controller = pid(Kp, Ki, Kd);

% Closed-Loop System with PID Controller
sys_cl = feedback(pid_controller * sys_tf, 1);

% Plot Step Response of Closed-Loop System
figure;
step(sys_cl);
title('Step Response of the Closed-Loop System with PID Controller');
grid on;

% Calculate Performance Metrics for Closed-Loop System
info_cl = stepinfo(sys_cl);

% Display Performance Metrics for Closed-Loop System
disp('Closed-Loop Settling Time: '), disp(info_cl.SettlingTime);
disp('Closed-Loop Overshoot: '), disp(info_cl.Overshoot);
disp('Closed-Loop Peak Time: '), disp(info_cl.PeakTime);
disp('Closed-Loop Steady State Value: '), disp(info_cl.SettlingMin);  % Assuming settling value

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% MATLAB Code for Frequency-Based Analysis

% Open-Loop System Bode Plot
figure;
bode(sys_tf);
title('Bode Plot of the Open-Loop System');
grid on;

% Finding the Cutoff Frequency for Open-Loop System
[mag, phase, w] = bode(sys_tf);
mag_db = 20*log10(squeeze(mag));
cutoff_freq_open = w(find(mag_db <= -3, 1));  % -3 dB cutoff frequency

% Closed-Loop System Bode Plot
figure;
bode(sys_cl);
title('Bode Plot of the Closed-Loop System with PID Controller');
grid on;

% Finding the Cutoff Frequency for Closed-Loop System
[mag_cl, phase_cl, w_cl] = bode(sys_cl);
mag_cl_db = 20*log10(squeeze(mag_cl));
cutoff_freq_closed = w_cl(find(mag_cl_db <= -3, 1));  % -3 dB cutoff frequency

% Gain Margin (GM) and Phase Margin (PM) for Closed-Loop System
[GM, PM, Wcg, Wcp] = margin(sys_cl);

% Display Cutoff Frequencies and Margins
disp('Cutoff Frequency (Open-Loop): '), disp(cutoff_freq_open);
disp('Cutoff Frequency (Closed-Loop): '), disp(cutoff_freq_closed);
disp('Gain Margin (GM): '), disp(GM);
disp('Phase Margin (PM): '), disp(PM);

% Sinusoidal Input Response Comparison
t = 0:0.01:10;
input_signal_1hz = 5 + sin(2*pi*1*t);
input_signal_cutoff = 5 + sin(2*pi*cutoff_freq_closed*t);

% Open-Loop Response to 1 Hz Sinusoidal Input
output_open_1hz = lsim(sys_tf, input_signal_1hz, t);

% Open-Loop Response to Cutoff Frequency Sinusoidal Input
output_open_cutoff = lsim(sys_tf, input_signal_cutoff, t);

% Closed-Loop Response to 1 Hz Sinusoidal Input
output_closed_1hz = lsim(sys_cl, input_signal_1hz, t);

% Closed-Loop Response to Cutoff Frequency Sinusoidal Input
output_closed_cutoff = lsim(sys_cl, input_signal_cutoff, t);

% Plotting Sinusoidal Responses
figure;
subplot(2,1,1);
plot(t, input_signal_1hz, 'b', t, output_open_1hz, 'r', t, output_closed_1hz, 'g');
legend('Input Signal (1 Hz)', 'Open-Loop Response', 'Closed-Loop Response');
title('System Response to 1 Hz Sinusoidal Input');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

subplot(2,1,2);
plot(t, input_signal_cutoff, 'b', t, output_open_cutoff, 'r', t, output_closed_cutoff, 'g');
legend('Input Signal (Cutoff Frequency)', 'Open-Loop Response', 'Closed-Loop Response');
title('System Response to Cutoff Frequency Sinusoidal Input');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

