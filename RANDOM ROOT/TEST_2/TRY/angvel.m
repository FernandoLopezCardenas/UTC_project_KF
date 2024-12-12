clc
clear all
close all
%%%%%%%%%%%%%%%%%%%%%%%%%%%% Registration data %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Leer todo el archivo CSV
data = readmatrix('all_logs.csv');

%%%%%%%%%%%%%%%%%%%%%%%%%%%% Quaternions angles %%%%%%%%%%%%%%%%%%%%%%%%%%%
% Extraer datos de los quaterniones desde la matriz 'data'
q_x = data(:, 11);  % Componente X del quaternion
q_y = data(:, 12);  % Componente Y del quaternion
q_z = data(:, 13);  % Componente Z del quaternion
q_w = data(:, 10);  % Componente W del quaternion

% Construir la matriz de quaterniones [w, x, y, z]
quaternions = [q_w, q_x, q_y, q_z];

% Extraer derivadas de los quaterniones
dq_x = data(:, 18);  % Derivada de X
dq_y = data(:, 19);  % Derivada de Y
dq_z = data(:, 20);  % Derivada de Z
dq_w = data(:, 17);  % Derivada de W

% Construir la matriz de derivadas de los quaterniones [dw, dx, dy, dz]
quaternionsDot = [dq_w, dq_x, dq_y, dq_z];

% Calcular las velocidades angulares a partir de los quaterniones y sus derivadas
angularVelocities = processQuaternionDatabase(quaternions, quaternionsDot);

% Mostrar las velocidades angulares
disp('Velocidades angulares (rad/s):');
disp(angularVelocities);

% Guardar las velocidades angulares en un archivo si es necesario
% save('angularVelocities.mat', 'angularVelocities');




