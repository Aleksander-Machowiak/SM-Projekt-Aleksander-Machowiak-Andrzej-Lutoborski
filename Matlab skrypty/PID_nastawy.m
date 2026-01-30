clc; clear; close all;

% Wyznaczone parametry modelu (1 rzad + delay)
K = 22.4; Tau = 281; Delay = 14.8; T_offset = 19.8;

s = tf('s');
G = (K/(Tau*s+1))*exp(-Delay*s);

%% Automatyczny tuning
C = pidtune(G,'PID');   % stroi PI
% Kp = C.Kp; Ki = C.Ki; Kd = C.Kd; 
% Otrzymane wartości Kp = 0.0647,  Ki = 4.0711e-04, Kd = 0

%% Ręczny z użyciem pidTuner
% pidTuner(G, 'PID')
 Kp = 0.29304; Ki = 0.0010465; Kd = 0;
% Kd = 1.0431;


