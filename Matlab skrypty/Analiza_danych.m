clc; clear; close all;

D = readmatrix("odp_skok_30min.csv");

t = D(:,1);     % czas [s]
u = D(:,2);     % wymuszenie 0..1
T = D(:,3);     % temperatura [°C]

%% Pierwsze 60s 

% idx = t <= 60;
% 
% figure;
% yyaxis left
% plot(t(idx), u(idx), 'LineWidth', 1.5);
% ylabel("u (PWM)")
% ylim([0 1])
% grid on
% 
% yyaxis right
% plot(t(idx), T(idx), 'LineWidth', 1.5);
% ylabel("Temperatura [°C]")
% 
% xlabel("Czas [s]")
% title("Odpowiedź obiektu – pierwsze 60 s")

%% Wykres 30 min
% 
% figure;
% 
% plot(t, T, 'LineWidth', 1.5);
% ylabel("Temperatura [°C]")
% xlabel("Czas [s]")
% title("Odpowiedź obiektu – pełny przebieg 30 minut")
% 
% fprintf("czas: %.1f .. %.1f s, N=%d\n", t(1), t(end), numel(t));
% fprintf("T: min=%.2f max=%.2f\n", min(T), max(T));

%% Wyznaczenie modelu i porównanie z pomiarami

tStep = 15;

u0 = mean(u(t < tStep));
T0 = mean(T(t < tStep));

u_of = u - u0;      % wejście jako offset
y_of = T - T0;      % wyjście jako offset

Ts = mean(diff(t));  % czas próbkowania

% Dane
data = iddata(y_of, u_of, Ts);

% Model: 1 rząd + opóźnienie
m = procest(data, "P1D");

K     = m.Kp;
Tau   = m.Tp1;
Delay = m.Td;

fprintf("K=%.3g, Tau=%.3g s, Delay=%.3g s\n", K, Tau, Delay);

% odpowiedź modelu na to samo u(t) 
y_model_dev = lsim(m, u_of, t);
T_model = y_model_dev + T0;
err = T - T_model;

figure;

subplot(2,1,1);
plot(t, T, 'b.', 'MarkerSize', 6); hold on
plot(t, T_model, 'g-', 'LineWidth', 1.5);
ylabel("Temperatura [°C]");
legend("pomiar", "model", "Location", "best");

subplot(2,1,2);
plot(t, err, 'b-'); grid on;
xlabel("Czas [s]");
ylabel("Błąd modelu [°C]");