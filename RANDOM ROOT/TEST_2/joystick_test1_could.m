clear all
close all
clc

%%%%%%%%%%%%%%%%%%%%%%% Registration data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Leer todo el archivo CSV
data = readmatrix('joystick_test1_newDef.csv');

%Extract reference data from X,Y,Theta, DELTA, Throttle
ber_mea_x = data(:, 32);                    % Using X as X (AF in Excel)
ber_mea_y = -data(:, 34);                   % Using Z as Y (AH in Excel)
ber_mea_theta = data(:,25);                 % Using θ (Y in Excel)
ber_mea_DELTA = data(:,11);                 % Using δ as Steering (K in Excel)
ber_mea_throttle = data(:,22);              % Using Throttle (V in Excel)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%% Mi referencia es el Marconi_computed_path NO ESTE %%%%%%%%%%%
% Filter non-NaN entries measured data from X,Y,Theta
ber_mea_x = ber_mea_x(~isnan(ber_mea_x));
ber_mea_y = ber_mea_y(~isnan(ber_mea_y));
ber_mea_theta = ber_mea_theta(~isnan(ber_mea_theta));
ber_mea_DELTA = ber_mea_DELTA(~isnan(ber_mea_DELTA));
ber_mea_throttle = ber_mea_throttle(~isnan(ber_mea_throttle));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%% Mi referencia es el Marconi_computed_path %%%%%%%%%%%%%%%%
load("Marconi_computed_path.mat");
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%% Gaussian noise %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parámetros
n = length(ber_mea_x); % Número de muestras (no +1 aquí)
mu = 0;     % Media
sigma = 0.01; % Desviación estándar (ajustada para que esté en [-1, 1])             %%%%%%%%%%%%% 0.1

% Generar ruido gaussiano
ruido_gaussiano = mu + sigma * randn(3, n);

% Limitar el ruido gaussiano al rango [-1, 1]
ruido_gaussiano(ruido_gaussiano < -1) = -1;
ruido_gaussiano(ruido_gaussiano > 1) = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Initial values %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Initial position
Xk = [x(1); y(1); 0];

%Measured poses without noise
% X = [ber_mea_x'; ber_mea_y'; ber_mea_theta'];
% %Measured poses with noise
X = [ber_mea_x'; ber_mea_y'; ber_mea_theta'] + ruido_gaussiano;

% Length between wheels
L = 0.155;

% %Covariance associated with the noise
% Qk = [1 0 0;0 1 0;0 0 1]*850;
%Initialize the Covariance associated with the system
Pk = [1 0 0;0 1 0;0 0 1]*10;
% %Variance associated
% Rk = [1 0 0;0 1 0;0 0 1]*200;

% IF Qk >> Rk The filter relies more on model predictions and less on measurements.
% IF Rk >> Qk The filter relies more on measurements and less on model predictions.


Id = eye(3,3);
Hk = eye(3,3);

ti = 0;
tf = length(X);
t = linspace(ti,tf,length(X));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% DATA LOSS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% n_ran = randi([10,100]);
% for m = 1 : n_ran
%     random = randi([1,length(X)]);
%     %Measured poses
%     X(:,random) = X(:,random).*zeros(3,1);
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%% Initialisation throttle %%%%%%%%%%%%%%%%%%%%%%%%%%
% Mean throttle
mean_val = mean(ber_mea_throttle);
% Desviación estándar para centrar los valores alrededor de la media
sigma2 = (max(ber_mea_throttle) - min(ber_mea_throttle)) / 6; 
% Aproximación para que el 99.7% esté en [min, max]
% Generar valores aleatorios con media y rango deseados en un vector
Dk = mean_val + sigma2 * randn(1, length(X));
% Limitar los valores generados al rango [min_val, max_val]
Dk(Dk < min(ber_mea_throttle)) = min(ber_mea_throttle);
Dk(Dk > max(ber_mea_throttle)) = max(ber_mea_throttle);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%% Initialisation DELTA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Mean throttle
mean_val = mean(ber_mea_DELTA);
% Desviación estándar para centrar los valores alrededor de la media
sigma3 = (max(ber_mea_DELTA) - min(ber_mea_DELTA)) / 6; 
% Aproximación para que el 99.7% esté en [min, max]
% Generar valores aleatorios con media y rango deseados en un vector
DELTA = mean_val + sigma3 * randn(1, length(X));
% Limitar los valores generados al rango [min_val, max_val]
DELTA(DELTA < min(ber_mea_DELTA)) = min(ber_mea_DELTA);
DELTA(DELTA > max(ber_mea_DELTA)) = max(ber_mea_DELTA);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%% Extended Kalman Filter %%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1: length(X)
%%%%%%%%%%%%%%%%%%%%%%%% In case of losing data %%%%%%%%%%%%%%%%%%%%%%%%%%%
    if X(:,i)==0
        %Covariance associated with the noise
        Qk = [1 0 0;0 1 0;0 0 1]*15;
        %Variance associated
        Rk = [1 0 0;0 1 0;0 0 1]*99999;
    else
        %Covariance associated with the noise
        Qk = [1 0 0;0 1 0;0 0 1]*1500;
        %Variance associated
        Rk = [1 0 0;0 1 0;0 0 1]*200;
    end

    % IF Qk >> Rk The filter relies more on measurements and less on model predictions.
    % IF Rk >> Qk The filter relies more on model predictions and less on measurements.

    % Qk > Rk
    % Si mi incertidumbre en mi modelo en el instante K > incertidumbre en
    % mi medida en el instante K, se confiara mas en mi medicion que en mi
    % modelo

    % Rk > Qk
    % Si la incertidumbre en la medida en el instante K > incertidumbre en
    % el modelo en el instante K, se confiara mas en el modelo que en la
    % medicion

%%%%%%%%%%%%%%%%%%%%% Update state from X,Y,Theta %%%%%%%%%%%%%%%%%%%%%%%%%
    %Update the next X
    Xk(1,i+1) = Xk(1,i) + Dk(i)*cos(DELTA(i))*cos(Xk(3,i));
    %Update the next Y
    Xk(2,i+1) = Xk(2,i) + Dk(i)*cos(DELTA(i))*sin(Xk(3,i));
    %Update the next theta
    Xk(3,i+1) = Xk(3,i) + Dk(i)/L *sin(DELTA(i));
    %These values are shifted to the 2nd column
    %Since the fist contains the inicial values of X
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PREDICT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Jacobian of the state matrix compared
    %to the a priori estimate 
    Fk=[1 0 -Dk(i)*cos(DELTA(i))*sin(Xk(3,i))
        0 1 Dk(i)*cos(DELTA(i))*cos(Xk(3,i))
        0 0 1];

    %Predicted estimate covariance
    Pk = Fk*Pk*Fk'+Qk;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Keep observable X,Y,Theta
    Zk = X(:,i);
    %Keep value estimated from X,Y,Theta
    Zest = Xk(:,i+1);
    %The estimated measure taking into account the prediction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% UPDATE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Innovation on measurement pre-fit residual
    Yk = Zk - Hk*Zest;  % La diferencia completa de las observaciones
    %Innovation covariance
    Sk = Hk*Pk*Hk'+Rk;
    %Optimal Kalman gain
    Kk = Pk*Hk'*inv(Sk);
    %Update state estimate
    Xk(:,i+1) = Xk(:,i+1) + Kk*Yk;
    %Updated estimated covariance
    Pk = (Id - Kk*Hk)*Pk;

    %
    sigmax(i)=sqrt(Pk(1,1));
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Xk;
Pk;

%Error in X%
Ex = sqrt((Xk(1,2:2391) - X(1,:)).^2 + (Xk(2,2:2391) - X(2,:)).^2);
%Summation of squared errors
Err = sqrt((Xk(1,2:2391)-X(1,:)).^2 + (Xk(2,2:2391)-X(2,:)).^2);
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
% %State estimate
plot(Xk(1,:),Xk(2,:),'r','LineWidth',1.5)
%Measurement
plot(X(1,:),X(2,:),'--g','LineWidth',2)
% plot the circuit (pov from the ground station)
plot(x, y,'m','LineWidth',1.5);
% %Ground truth
% plot(gt(1,:),gt(2,:),'-.b','LineWidth',1.5)
legend('EKF','Measurement','Ideal','Location','north')
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
legend('X estimate','X measurement','Location','north')
xlabel('Time')
ylabel('X value')
title('Graph of X-time')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% Graph of Y's respect the time %%%%%%%%%%%%%%%%%%%%%%%%
subplot(2,1,2)
hold on
%State estimate
plot(linspace(ti,tf,length(Xk)),Xk(2,:),'r','LineWidth',1.5)
%Measurement
plot(t,X(2,:),'--g','LineWidth',1.5)
legend('X estimate','X measurement','Location','southwest')
xlabel('Time')
ylabel('Y value')
title('Graph of Y-time')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
subplot(2,1,1)
hold on
% plot the circuit (pov from the ground station)
plot(x,'m','LineWidth',1.5);
subplot(2,1,2)
% plot the circuit (pov from the ground station)
plot(y,'m','LineWidth',1.5);