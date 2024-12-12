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

%%%%%%%%%%%%%%%%%%%%%%%% Extended Kalman Filter %%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1: length(X)
%%%%%%%%%%%%%%%%%%%%%%%% In case of losing data %%%%%%%%%%%%%%%%%%%%%%%%%%%
    if X(:,i)==0
        %Covariance associated with the noise
        Qk = [1 0 0;0 1 0;0 0 1]*1500;
        %Variance associated
        Rk = [1 0 0;0 1 0;0 0 1]*1e10;
    else
        %Covariance associated with the noise
        Qk = [1 0 0;0 1 0;0 0 1]*1500;
        %Variance associated
        Rk = [1 0 0;0 1 0;0 0 1]*2500;
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % %Covariance associated with the noise
    % Qk = [1 0 0;0 1 0;0 0 1]*1500;
    % %Variance associated
    % Rk = [1 0 0;0 1 0;0 0 1]*200;
    % 
    % IF Qk >> Rk The filter relies more on measurements and less on model predictions.
    % IF Rk >> Qk The filter relies more on model predictions and less on measurements.

    % Qk > Rk
    %   If my uncertainty in my model at time K > uncertainty in my
    %   measurement at time K, my measurement will be trusted more than my
    %   model.

    % Qk < Rk
    %   If my uncertainty in my model at time K < uncertainty in my
    %   measurement at time K, the model will be trusted more than the measurement.

%%%%%%%%%%%%%%%%%%%%% Update state from X,Y,Theta %%%%%%%%%%%%%%%%%%%%%%%%%
    %Update the next X
    Xk(1,i+1) = Xk(1,i) + Dk(i)*cos(Xk(3,i));
    %Update the next Y
    Xk(2,i+1) = Xk(2,i) + Dk(i)*sin(Xk(3,i));
    %Update the next theta
    Xk(3,i+1) = Xk(3,i) + w(i,3);
    %These values are shifted to the 2nd column
    %Since the fist contains the inicial values of X
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PREDICT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Jacobian of the state matrix compared
    %to the a priori estimate 
    Fk=[1 0 -Dk(i)*sin(Xk(3,i))
        0 1 Dk(i)*cos(Xk(3,i))
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
