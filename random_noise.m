clc
clear all
close all

% Parámetros
n = 1000; % Número de muestras

%% Ruido Gaussiano
mu = 0;     % Media
sigma = 0.5; % Desviación estándar (ajustada para que esté en [-1, 1])

% Generar ruido gaussiano
ruido_gaussiano = mu + sigma * randn(n, 1);

% Limitar el ruido gaussiano al rango [-1, 1]
ruido_gaussiano(ruido_gaussiano < -1) = -1;
ruido_gaussiano(ruido_gaussiano > 1) = 1;

%% Ruido Uniforme
% Generar ruido uniforme en el rango [-1, 1]
ruido_uniforme = -1 + 2 * rand(n, 1);

%% Ruido de Poisson
lambda = 0; % La media debe ser 0 para estar en [-1, 1]
ruido_poisson = poissrnd(lambda, n, 1);

% Limitar el ruido de Poisson al rango [-1, 1] (0 o -1)
ruido_poisson(ruido_poisson > 1) = 1;
ruido_poisson(ruido_poisson < -1) = -1; % Esto no cambiará nada, ya que el ruido de Poisson es >= 0.

%% Mostrar los histogramas
figure;

subplot(3, 1, 1);
histogram(ruido_gaussiano, 30);
title('Ruido Gaussiano');
xlabel('Valor');
ylabel('Frecuencia');
xlim([-1 1]);

subplot(3, 1, 2);
histogram(ruido_uniforme, 30);
title('Ruido Uniforme');
xlabel('Valor');
ylabel('Frecuencia');
xlim([-1 1]);

subplot(3, 1, 3);
histogram(ruido_poisson, 30);
title('Ruido de Poisson');
xlabel('Valor');
ylabel('Frecuencia');
xlim([-1 1]);

