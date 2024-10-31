clear all
close all
clc
grid

%Initial position
X = [2;1;pi/2];
%Measured poses
% X = [2.0244 1.9585 1.9208;
%      1.4679 1.9892 2.2414;
%      1.6707 1.6815 2.1821];

%Landmark
L = [5 4 1;0.5 3 1];
%Distances (Observation model)
d1 = [3.1623; 3.3977; 3.5382];    % the measurements of distance to the landmark 1
d2 = [2.5000; 2.2819; 2.2100];     % the measurements of distance to the landmark 2
d3 = [1.1180; 1.3776; 1.5504];     % the measurements of distance to the landmark 3

%Covariance associated with the noise
Qk = [1 0 0;0 1 0;0 0 1]*2;
%Initialize the Covariance associated with the system
Pk = [10 0 0;0 10 0;0 0 1];
%Variance associated
Rk = [0.1 0 0;0 0.1 0;0 0 0.1];
%Elementary displacement
Dk=[0.5;0.5;0.25]*0.1;
%Elementary rotation
Wk=[0.1;0.01;0.5];

hold on
axis([0 5 0 5])
%plot the landmarks
plot(L(1,1),L(2,1),'*r')
plot(L(1,2),L(2,2),'*r')
plot(L(1,3),L(2,3),'*r')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Observation matrix
%Hk =[1 0 0;0 1 0;0 0 1];
%State Matrix
%Fk=[1 0 0;0 1 0;0 0 1];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Identity Matrix
Id=eye(length(Dk));
%Ground truth
gt= [2.0000 2.0000 1.9501 1.9226
     1.0000 1.5000 1.9975 2.2460
     1.5708 1.6708 1.6808 2.1808];

for i = 1: length(Dk)
    %Update the next X
    X(1,i+1) = X(1,i) + Dk(i)*cos(X(3,i));
    %Update the next Y
    X(2,i+1) = X(2,i) + Dk(i)*sin(X(3,i));
    %Update the next theta
    X(3,i+1) = X(3,i) + Wk(i);
    %These values are shifted to the 2nd column
    %Since the fist contains the inicial values of X [2,1,pi/2]

%% %%%%%%%%%%%%%%%%%%%%%%%%% PREDICT %%%%%%%%%%%%%%%%%%%%%%%%%
    Fk=[1 0 -Dk(i)*sin(X(3,i))
        0 1 Dk(i)*cos(X(3,i))
        0 0 1];
    %Jacobian of the state matrix compared
    %to the a priori estimate 

    %Predicted estimate covariance
    Pk = Fk*Pk*Fk'+Qk;
%% %%%%%%%%%%%%%%%%%%%%%%%%% UPDATE %%%%%%%%%%%%%%%%%%%%%%%%%
    %Z value measurement
    Z = [d1(i);d2(i);d3(i)];
    %Z value estimate = sqrt((Xest - Xmea)^2 + (Yest - Ymea)^2)
    Zest = [sqrt((X(1,i+1)-L(1,1))^2+(X(2,i+1)-L(2,1))^2);
            sqrt((X(1,i+1)-L(1,2))^2+(X(2,i+1)-L(2,2))^2);
            sqrt((X(1,i+1)-L(1,3))^2+(X(2,i+1)-L(2,3))^2)];
    %The estimated measure taking into account the prediction
    %For each landmark

    Hk = [(X(1,i+1)-L(1,1))/Zest(1) (X(2,i+1)-L(2,1))/Zest(1) 0
          (X(1,i+1)-L(1,2))/Zest(2) (X(2,i+1)-L(2,2))/Zest(2) 0
          (X(1,i+1)-L(1,3))/Zest(3) (X(2,i+1)-L(2,3))/Zest(3) 0];
    %Jacobian of the difference between landmarks and the predicted
    %state estimate compared to the observation matrix estimate
    
    %Innovation on measurement pre-fit residual
    Yk = Z - Zest;
    %Innovation covariance
    Sk = Hk*Pk*Hk'+Rk;
    %Optimal Kalman gain
    Kk = Pk*Hk'*Sk^-1;
    %Update state estimate
    X(:,i+1) = X(:,i+1) + Kk*Yk; %Corrected pose
    %Updated estimated covariance
    Pk = (Id - Kk*Hk)*Pk

    %
    sigmax(i)=sqrt(Pk(1,1));
end

hold on
plot(gt(1,:),gt(2,:),'.b')
plot(X(1,:),X(2,:),'.r')
Ex = sqrt((X(1,:) - gt(1,:)).^2 + (X(2,:) - gt(2,:)).^2);
mean(Ex)

figure
plot(Ex(2:4))
hold on
plot(sigmax,'r')
plot(-sigmax,'r')
