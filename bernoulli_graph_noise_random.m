clear all
close all
clc
grid

%%%%%%%%%%%%%%%%%%%%%%% Registration data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Leer todo el archivo CSV
data = readmatrix('bernoulli_old_PD.csv');

% Extract measured data from X,Y,Theta
ber_mea_x = data(:, 19);
ber_mea_y = data(:, 20);
ber_mea_theta = data(:,21);
%Extract reference data from X,Y,Theta
ber_gt_x = data(:, 25);
ber_gt_y = data(:, 26);
ber_gt_theta = data(:,27);

% Filter non-NaN entries measured data from X,Y,Theta
ber_mea_x = ber_mea_x(~isnan(ber_mea_x));
ber_mea_y = ber_mea_y(~isnan(ber_mea_y));
ber_mea_theta = ber_mea_theta(~isnan(ber_mea_theta)); 
% Filter non-NaN entries reference data from X,Y,Theta
ber_gt_x = ber_gt_x(~isnan(ber_gt_x)); 
ber_gt_y = ber_gt_y(~isnan(ber_gt_y)); 
ber_gt_theta = ber_gt_theta(~isnan(ber_gt_theta)); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Ruido Uniforme
% Parámetros
n = length(ber_mea_x); % Número de muestras (no +1 aquí)
% Generar ruido uniforme en el rango [-0.005, 0.005]
ruido_uniforme = -0.1 + (0.1 - (-0.1)) * rand(n, 3); % 3 columnas para X, Y, Theta

%Initial position
Xk = [ber_gt_x(1); ber_gt_y(1); ber_gt_theta(1)];

%%Measured poses
X = [ber_mea_x'; ber_mea_y'; ber_mea_theta'] + ruido_uniforme';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% DATA LOSS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
n_ran = randi([1,20]);
for m = 1 : n_ran
    random = randi([1,length(X)]);
    %%Measured poses
    X(:,random) = X(:,random).*zeros(3,1);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%Covariance associated with the noise
Qk = [1 0 0;0 1 0;0 0 1]*0.08;
%Initialize the Covariance associated with the system
Pk = [1 0 0;0 1 0;0 0 1]*0.1;
%Variance associated
Rk = [1 0 0;0 1 0;0 0 1]*13;
%Observation matrix
Hk =[1 0 0;0 1 0;0 0 1];
%State Matrix
Fk=[1 0 0;0 1 0;0 0 1];
%Identity Matrix
Id=[1 0 0;0 1 0;0 0 1];
%Ground truth
gt= [ber_gt_x(1) ber_gt_x';
     ber_gt_y(1) ber_gt_y';
     ber_gt_theta(1) ber_gt_theta'];

for i = 1: length(X)
%%%%%%%%%%%%%%%%%%%%% Update state from X,Y,Theta %%%%%%%%%%%%%%%%%%%%%%%%%
    Xk(:, i + 1) = Xk(:, i);
%%%%%%%%%%%%%%%%%%%%%%%%% PREDICT %%%%%%%%%%%%%%%%%%%%%%%%%
    %Predicted estimate covariance
    Pk = Fk*Pk*Fk'+Qk;
%%%%%%%%%%%%%%%%%%%%%%%%% UPDATE %%%%%%%%%%%%%%%%%%%%%%%%%
    %Keep value estimated from X,Y,Theta
    Zest = Xk(:, i + 1);
    %Keep observable X,Y,Theta
    Zk = X(:,i);
    %Innovation on measurement pre-fit residual
    Yk = Zk - Hk*Zest;
    %Innovation covariance
    Sk = Hk*Pk*Hk'+Rk;
    %Optimal Kalman gain
    Kk = Pk*Hk'*Sk^-1;
    %Update state estimate
    Xk(:,i+1) = Xk(:,i+1) + Kk*Yk ;%+ ruido_uniforme(i,:)'; % Usar ruido de la fila i;
    %Updated estimated covariance
    Pk = (Id - Kk*Hk)*Pk;

    %
    sigmax(i)=sqrt(Pk(1,1));
end

Xk
Pk

%Error in X%
Ex = Xk(1,:)-gt(1,:);
%Summation of squared errors
Err = sqrt((Xk(1,:)-gt(1,:)).^2 + (Xk(2,:)-gt(2,:)).^2);
%Mean of the summation of squared errors
mean(Err)

hold on
%State estimate
plot(Xk(1,:),Xk(2,:),'r','LineWidth',1.5)
%Measurement
plot(X(1,:),X(2,:),'--g')
%Ground truth
plot(gt(1,:),gt(2,:),'-.b')
figure
%Error graph in X since row 2 to row 4
plot(Ex)
hold on
plot(sigmax,'r')
plot(-sigmax,'r')
