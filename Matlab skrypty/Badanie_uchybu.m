%% Analiza uchybu z log_live_uchyb.csv

clear; clc; close all;

fname = "log_live_uchyb.csv";
M = readmatrix(fname);
if size(M,2) < 5
    error("Plik ma %d kolumn. Oczekuję min. 5: t, u, T, Tzad, e.", size(M,2));
end

t    = M(:,1);
e = M(:,5);


%% Metryki uchybu (cały przebieg)
absE = abs(e);
MAE  = mean(absE, 'omitnan');                     % Mean Absolute Error
RMSE = sqrt(mean(e.^2, 'omitnan'));               % Root Mean Square Error
Emax = max(absE);                                 % max |e|
bias = mean(e, 'omitnan');                        % średni uchyb (ze znakiem)

%% Uchyb ustalony: średnia z końcówki (np. ostatnie 20% próbek)
N = numel(t);
tailFrac = 0.20;
idx0 = max(1, floor((1-tailFrac)*N));
e_ss = mean(e(idx0:end), 'omitnan');
e_ss_abs = mean(abs(e(idx0:end)), 'omitnan');


%% Wydruk wyników
fprintf("\n=== METRYKI UCHYBU ===\n");
fprintf("MAE        = %.4f [degC]\n", MAE);
fprintf("RMSE       = %.4f [degC]\n", RMSE);
fprintf("max|e|     = %.4f [degC]\n", Emax);
fprintf("mean(e)    = %.4f [degC]\n", bias);
fprintf("e_ss (ostatnie %.0f%%)        = %.4f [degC]\n", 100*tailFrac, e_ss);
fprintf("mean|e| (ostatnie %.0f%%)     = %.4f [degC]\n", 100*tailFrac, e_ss_abs);


%% Wykres
plot(t, e, 'LineWidth', 1.2); hold on;
yline(0, '--');
grid on;
ylabel('Uchyb e [°C]');
xlabel('Czas t [s]');