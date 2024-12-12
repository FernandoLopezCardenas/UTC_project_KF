clear all
close all
clc
grid

%%%%%%%%%%%%%%%%%%%%%%% Registration data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Leer todo el archivo CSV
data = readmatrix('joystick_test1_newDef.csv');

% Extract measured data from X,Y,Theta
ber_mea_x = data(:, 32);
ber_mea_y = -data(:, 34);
ber_mea_theta = data(:,25);

%Extract reference data from X,Y,Theta
ber_gt_x = data(:, 19);
ber_gt_y = -data(:, 21);
ber_gt_theta = data(:,12);

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
ruido_uniforme = -0.2 + (0.2 - (-0.2)) * rand(n, 3); % 3 columnas para X, Y, Theta

%Initial position
Xk = [ber_gt_x(1); ber_gt_y(1); ber_gt_theta(1)];

%%Measured poses without noise
X = [ber_mea_x'; ber_mea_y'; ber_mea_theta'];
%%Measured poses with noise
X = [ber_mea_x'; ber_mea_y'; ber_mea_theta'] + ruido_uniforme';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% DATA LOSS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
n_ran = randi([1,20]);
for m = 1 : n_ran
    random = randi([1,length(X)]);
    %Measured poses
    X(:,random) = X(:,random).*zeros(3,1);
end
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ti = 0;
tf = length(X);
t = linspace(ti,tf,length(X));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %Landmark
% L = [5 4 1;0.5 3 1];
% %Distances (Observation model)
% d1 = [3.1623; 3.3977; 3.5382];    % the measurements of distance to the landmark 1
% d2 = [2.5000; 2.2819; 2.2100];     % the measurements of distance to the landmark 2
% d3 = [1.1180; 1.3776; 1.5504];     % the measurements of distance to the landmark 3
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Covariance associated with the noise
Qk = [1 0 0;0 1 0;0 0 1]*0.08;
%Initialize the Covariance associated with the system
Pk = [1 0 0;0 1 0;0 0 1]*10;
%Variance associated
Rk = [1 0 0;0 1 0;0 0 1]*15;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %Covariance associated with the noise
% Qk = [1 0 0;0 1 0;0 0 1]*20;
% %Initialize the Covariance associated with the system
% Pk = [1 0 0;0 1 0;0 0 1]*10;
% %Variance associated
% Rk = [1 0 0;0 1 0;0 0 1]*0.1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Elementary displacement
Dk=0.001;
%Elementary rotation
Wk=0.01;

% hold on
% axis([0 5 0 5])
% %plot the landmarks
% plot(L(1,1),L(2,1),'*r')
% plot(L(1,2),L(2,2),'*r')
% plot(L(1,3),L(2,3),'*r')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %Observation matrix
% %Hk =[1 0 0;0 1 0;0 0 1];
% %State Matrix
% %Fk=[1 0 0;0 1 0;0 0 1];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Identity Matrix
Id=[1 0 0;0 1 0;0 0 1];
%Ground truth
gt= [ber_gt_x(1) ber_gt_x';
     ber_gt_y(1) ber_gt_y';
     ber_gt_theta(1) ber_gt_theta'];

for i = 1: length(X)
%%%%%%%%%%%%%%%%%%%%% Update state from X,Y,Theta %%%%%%%%%%%%%%%%%%%%%%%%%
    %Update the next X
    Xk(1,i+1) = Xk(1,i) + Dk*cos(Xk(3,i));
    %Update the next Y
    Xk(2,i+1) = Xk(2,i) + Dk*sin(Xk(3,i));
    %Update the next theta
    Xk(3,i+1) = Xk(3,i) + Wk;
    %These values are shifted to the 2nd column
    %Since the fist contains the inicial values of X

%%%%%%%%%%%%%%%%%%%%%%%%% PREDICT %%%%%%%%%%%%%%%%%%%%%%%%%
    %Jacobian of the state matrix compared
    %to the a priori estimate 
    Fk=[1 0 -Dk*sin(Xk(3,i))
        0 1 Dk*cos(Xk(3,i))
        0 0 1];

    %Predicted estimate covariance
    Pk = Fk*Pk*Fk'+Qk;
%%%%%%%%%%%%%%%%%%%%%%%%% UPDATE %%%%%%%%%%%%%%%%%%%%%%%%%
    %Keep observable X,Y,Theta
    Zk = X(:,i);
    %Keep value estimated from X,Y,Theta
    Zest = Xk(:, i + 1);
    %The estimated measure taking into account the prediction
    %For each landmark

    Hk = [1 0 -Dk*sin(X(3,i))
          0 1 Dk*cos(X(3,i))
          0 0 1];
    %Jacobian of the difference between landmarks and the predicted
    %state estimate compared to the observation matrix estimate


    %Innovation on measurement pre-fit residual
    Yk = Zk - Zest;  % La diferencia completa de las observaciones
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
Ex = sqrt((Xk(1,:) - gt(1,:)).^2 + (Xk(2,:) - gt(2,:)).^2);
%Summation of squared errors
Err = sqrt((Xk(1,:)-gt(1,:)).^2 + (Xk(2,:)-gt(2,:)).^2);
%Mean of the summation of squared errors
mean(Err)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Signal Graph %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
hold on
%State estimate
plot(Xk(1,:),Xk(2,:),'r','LineWidth',1.5)
%Measurement
plot(X(1,:),X(2,:),'--g','LineWidth',1.5)
%Ground truth
plot(gt(1,:),gt(2,:),'-.b','LineWidth',1.5)
legend('KF','Measurement','Ideal','Location','north')
title('Graph of X-time')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% Graph of X's respect the time %%%%%%%%%%%%%%%%%%%%%%%%
figure
subplot(2,1,1)
hold on
%State estimate
plot(linspace(ti,tf,length(Xk)),Xk(1,:),'r','LineWidth',1.5)
%Measurement
plot(linspace(ti,tf,length(Xk)-1),X(1,:),'--g','LineWidth',1.5)
%Ground truth
plot(linspace(ti,tf,length(Xk)),gt(1,:),'-.b','LineWidth',1.5)
legend('X estimate','X measurement','X ideal','Location','north')
title('Graph of X-time')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% Graph of Y's respect the time %%%%%%%%%%%%%%%%%%%%%%%%
subplot(2,1,2)
hold on
%State estimate
plot(linspace(ti,tf,length(Xk)),Xk(2,:),'r','LineWidth',1.5)
%Measurement
plot(t,X(2,:),'--g','LineWidth',1.5)
%Ground truth
plot(linspace(ti,tf,length(Xk)),gt(2,:),'-.b','LineWidth',1.5)
legend('X estimate','X measurement','X ideal','Location','southwest')
title('Graph of Y-time')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%figure
%Error graph in X since row 2 to row 4
% plot(Ex)
% hold on
% plot(sigmax,'r')
% plot(-sigmax,'r')
