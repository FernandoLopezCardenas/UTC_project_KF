clear all
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%% Registration data %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Leer todo el archivo CSV
data = readmatrix('all_logs_random_0-3999.csv');

% Extract measured data from X,Y,Theta
ber_mea_x = data(:, 14);                    % Using X (Excel)
ber_mea_y = data(:, 15);                    % Using Y (Excel)
dx = data(:,21);                            % Using X velocity component 
dy = data(:,22);                            % Using Y velocity component 
% Filter non-NaN entries measured data from X,Y,Theta
ber_mea_x = ber_mea_x(~isnan(ber_mea_x));
ber_mea_y = ber_mea_y(~isnan(ber_mea_y));
dx = dx(~isnan(dx));
dy = dy(~isnan(dy));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% velocity vector %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ber_mea_velocity = sqrt(dx.^2 + dy.^2);     % Using velocity (V in Excel)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%% Quaternions angles %%%%%%%%%%%%%%%%%%%%%%%%%%%
% Extract measured data from quaternions
q_x = data(:,11);                           % quaternion X
q_y = data(:,12);                           % quaternion Y 
q_z = data(:,13);                           % quaternion z
q_w = data(:,10);                           % quaternion w

dq_x = data(:,18);                          % Derivate quaternion X
dq_y = data(:,19);                          % Derivate quaternion Y 
dq_z = data(:,20);                          % Derivate quaternion z
dq_w = data(:,17);                          % Derivate quaternion w

% Filter non-NaN entries measured data from X,Y,Theta
q_x = q_x(~isnan(q_x));
q_y = q_y(~isnan(q_y));
q_z = q_z(~isnan(q_z));
q_w = q_w(~isnan(q_w));

dq_x = dq_x(~isnan(dq_x));
dq_y = dq_y(~isnan(dq_y));
dq_z = dq_z(~isnan(dq_z));
dq_w = dq_w(~isnan(dq_w));

% Construir la matriz de quaterniones [w, x, y, z]
quaternion = [q_w, q_x, q_y, q_z];
% Construir la matriz de derivadas de los quaterniones [dw, dx, dy, dz]
dquaternion = [dq_w, dq_x, dq_y, dq_z];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%% quaternions angles to degrees %%%%%%%%%%%%%%%%%%%%%
quat = quaternions(quaternion);         % Call quaternions function
ber_mea_theta = quat(:,3);                      % Just keep the yaw data

% Calcular las velocidades angulares a partir de los quaterniones y sus derivadas
w = processQuaternionDatabase(quaternion, dquaternion); % Angular velocity
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%% Gaussian noise %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
gaussian_noise = noise(length(ber_mea_x),0,3,0.1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%% Declaration of variables %%%%%%%%%%%%%%%%%%%%%%%%
%Initial position
Xk = [ber_mea_x(1);ber_mea_y(1); ber_mea_theta(1)];
%Measured poses without noise
%X = [ber_mea_x'; ber_mea_y'; ber_mea_theta'];
% %Measured poses with noise
 X = [ber_mea_x'; ber_mea_y'; ber_mea_theta'] + gaussian_noise;

%Initialize the Covariance associated with the system
Pk = [1 0 0;0 1 0;0 0 1]*10;

Id = eye(3,3);
Hk = eye(3,3);

ti = 0;
tf = length(X);
t = linspace(ti,tf,length(X));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% DATA LOSS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
X = data_loss(X,50,100);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Dk = 1/100 * ber_mea_velocity;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parámetros del filtro pasa bajas
Fc = 0.01;   % Frecuencia de corte normalizada (relativa a la frecuencia de Nyquist)
Fs = 100;     % Frecuencia de muestreo (se asume 1 Hz para simplificación)
[b, a] = butter(2, Fc, 'low');  % Filtro Butterworth de orden 2, pasa bajas

% Dentro del bucle del EKF
for i = 1:length(X)
    %%%%%%%%%%%%%%%%%%%%%%%% En caso de pérdida de datos %%%%%%%%%%%%%%%%%%%%%%%%%
    if X(:, i) == 0
        % Covarianza asociada al ruido
        Qk = [1 0 0; 0 1 0; 0 0 1] * 0.1^2;
        % Varianza asociada
        Rk = [1 0 0; 0 1 0; 0 0 1] * 1e10;
    else
        % Covarianza asociada al ruido
        Qk = [1 0 0; 0 1 0; 0 0 1] * 1500;
        % Varianza asociada
        Rk = [1 0 0; 0 1 0; 0 0 1] * 2500;
    end
    % Filtrar las mediciones X usando el filtro pasa bajas
    X_filtered = filter(b, a, X(:, 1:i), [], 2);  % Filtrado en cada iteración
    
    %%%%%%%%%%%%%%%%%%%%% Actualización del estado desde X, Y, Theta %%%%%%%%%%%%%%%%%%%%%
    % Actualización de las posiciones X, Y y theta usando los datos filtrados
    Xk(1, i + 1) = Xk(1, i) + Dk(i) * cos(Xk(3, i));
    Xk(2, i + 1) = Xk(2, i) + Dk(i) * sin(Xk(3, i));
    Xk(3, i + 1) = Xk(3, i) + w(i, 3);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PREDICCIÓN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Jacobiano de la matriz de estado
    Fk = [1 0 -Dk(i) * sin(Xk(3, i));
          0 1 Dk(i) * cos(Xk(3, i));
          0 0 1];

    % Covarianza de la estimación predicha
    Pk = Fk * Pk * Fk' + Qk;

    % Guardar X filtrado para usar en la actualización
    Zk = X_filtered(:, i);  % Usar el valor filtrado
    Zest = Xk(:, i + 1);    % Estimación del estado
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ACTUALIZACIÓN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Innovación en la medición
    Yk = Zk - Hk * Zest;
    % Covarianza de la innovación
    Sk = Hk * Pk * Hk' + Rk;
    % Ganancia óptima de Kalman
    Kk = Pk * Hk' / Sk;
    % Actualización del estado estimado
    Xk(:, i + 1) = Xk(:, i + 1) + Kk * Yk;
    % Covarianza estimada actualizada
    Pk = (Id - Kk * Hk) * Pk;

    sigmax(i) = sqrt(Pk(1, 1));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Xk;
Pk;

%Error in X%
Ex = sqrt((Xk(1,2:end) - X(1,:)).^2 + (Xk(2,2:end) - X(2,:)).^2);
%Summation of squared errors
Err = sqrt((Xk(1,2:end)-X(1,:)).^2 + (Xk(2,2:end)-X(2,:)).^2);
%Mean of the summation of squared errors
mean(Err)

figure
%Error graph in X since row 2 to row 4
plot(Ex)
hold on
plot(sigmax,'r')
plot(-sigmax,'r')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Signal Graph %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
hold on
grid
% %State estimate
plot(Xk(1,:),Xk(2,:),'r','LineWidth',1.5)
%Measurement
plot(X(1,:),X(2,:),'--g','LineWidth',2)
% % plot the circuit (pov from the ground station)
% plot(x, y,'m','LineWidth',1.5);
% %Ground truth
% plot(gt(1,:),gt(2,:),'-.b','LineWidth',1.5)
legend('EKF','Measurement','Location','southwest')
xlabel('X(m)')
ylabel('Y(m)')
title('EKF Trayectory')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% Graph of X's respect the time %%%%%%%%%%%%%%%%%%%%%%%%
figure
subplot(2,1,1)
hold on
%State estimate
plot(linspace(ti,tf,length(Xk)),Xk(1,:),'r','LineWidth',1.5)
%Measurement
plot(linspace(ti,tf,length(Xk)-1),X(1,:),'--g','LineWidth',1.5)
legend('X estimate','X measurement','Location','southwest')
xlabel('Time')
ylabel('X(m)')
title('Graph of X-time')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% Graph of Y's respect the time %%%%%%%%%%%%%%%%%%%%%%%%
subplot(2,1,2)
hold on
%State estimate
plot(linspace(ti,tf,length(Xk)),Xk(2,:),'r','LineWidth',1.5)
%Measurement
plot(t,X(2,:),'--g','LineWidth',1.5)
legend('Y estimate','Y measurement','Location','southwest')
xlabel('Time')
ylabel('Y(m)')
title('Graph of Y-time')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
