clc; clear; close all;

s = serialport("COM5",115200);
configureTerminator(s,"LF");
flush(s);

% Wartość zdana
writeline(s,"34")

figure;

h1 = subplot(3,1,1);
hpwm = plot(h1, NaN, NaN, 'LineWidth', 1.5);
grid on; ylabel("wymuszenie u (PWM)"); ylim([0 1]);

h2 = subplot(3,1,2);
ht = plot(h2, NaN, NaN, 'LineWidth', 1.5);
grid on; ylabel("Temperatura [°C]");
hold(h2,"on");
hsp = plot(h2, NaN, NaN, '--', 'LineWidth', 1.2);
legend(h2, {"T","SP"}, "Location","best");

h3 = subplot(3,1,3);
he = plot(h3, NaN, NaN, 'LineWidth', 1.5);
grid on; ylabel("uchyb e = SP - T [°C]"); xlabel("Czas [s]");
yline(h3, 0, '--');


% Dane do logowania 
% Texp = 15*60;     % 15 minut
% Ts   = 0.5;
% Nmax = Texp/Ts;   % Ilość próbek
Nmax = 800;  

t = zeros(Nmax,1);
u = zeros(Nmax,1);
T = zeros(Nmax,1);
SP = zeros(Nmax,1);
E  = zeros(Nmax,1); 

k = 1;
disp("Start logowania + wykres live");

while k <= Nmax
    line = strtrim(readline(s));

    if line == ""
        continue
    end

    % jeśli to nie jest telemetria CSV, pomiń (np. "OK S=35")
    if ~contains(line, ",")
        disp("RX cmd: " + line);   % opcjonalnie podgląd odpowiedzi
        continue
    end

    vals = sscanf(line,"%f,%f,%f,%f,%f");
        if numel(vals) ~= 5
        continue
    end


    t(k) = vals(1);
    u(k) = vals(2);
    T(k) = vals(3);
    SP(k) = vals(4);
    E(k) = vals(5);

    % aktualizacja wykresów
    set(hpwm,'XData',t(1:k),'YData',u(1:k));
    set(ht,'XData',t(1:k),'YData',T(1:k));
    set(hsp,'XData', t(1:k), 'YData', SP(1:k));
    set(he, 'XData',t(1:k),'YData',E(1:k));

    drawnow limitrate;
    k = k + 1;
end

disp("Koniec pomiaru");

data = [t(1:k-1) u(1:k-1) T(1:k-1) SP(1:k-1) E(1:k-1)];
writematrix(data,"log_live.csv");
