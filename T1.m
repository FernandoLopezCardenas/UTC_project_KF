clear all
close all
clc
grid

%Initial position
Xk = [2; 1; pi/2];

%%Measured poses
X = [2.0244 1.9585 1.9208;
     1.4679 1.9892 2.2414;
     1.6707 1.6815 2.1821]

%Covariance associated with the noise
Qk = [1 0 0;0 1 0;0 0 1]*0.1;
%Initialize the Covariance associated with the system
Pk = [10 0 0;0 10 0;0 0 1];
%Variance associated
Rk = [0.1 0 0;0 0.1 0;0 0 0.1];
%Observation matrix
Hk =[1 0 0;0 1 0;0 0 1];
%State Matrix
Fk=[1 0 0;0 1 0;0 0 1];
%Identity Matrix
Id=[1 0 0;0 1 0;0 0 1];
%Ground truth
gt= [2.0000    2.0000    1.9501    1.9226
     1.0000    1.5000    1.9975    2.2460
     1.5708    1.6708    1.6808    2.1808];

for i = 1: length(X)
    Xk(1,i+1) = Xk(1,i); %Update the next X
    Xk(2,i+1) = Xk(2,i); %Update the next Y
    Xk(3,i+1) = Xk(3,i); %Update the next theta

%%%%%%%%%%%%%%%%%%%%%%%%% PREDICT %%%%%%%%%%%%%%%%%%%%%%%%%
    %Predicted estimate covariance
    Pk = Fk*Pk*Fk'+Qk;
%%%%%%%%%%%%%%%%%%%%%%%%% UPDATE %%%%%%%%%%%%%%%%%%%%%%%%%
    %Z value estimated
    Zest = [Xk(1,i+1);Xk(2,i+1);Xk(3,i+1)];
    %Z observable
    Zk = X(:,i);
    %Innovation on measurement pre-fit residual
    Yk = Zk - Hk*Zest;
    %Innovation covariance
    Sk = Hk*Pk*Hk'+Rk;
    %Optimal Kalman gain
    Kk = Pk*Hk'*Sk^-1;
    %Update state estimate
    Xk(:,i+1) = Xk(:,i+1) + Kk*Yk;
    %Updated estimated covariance
    Pk = (Id - Kk*Hk)*Pk;

    %
    sigmax(i)=sqrt(Pk(1,1));
end

Xk
Pk

%Error in X%
Ex = Xk(1,:)-gt(1,:)
%Summation of squared errors
Err = sqrt((Xk(1,:)-gt(1,:)).^2 + (Xk(2,:)-gt(2,:)).^2)
%Mean of the summation of squared errors
mean(Err)

hold on
%State estimate
plot(Xk(1,:),Xk(2,:),'.r')
%Ground truth
plot(gt(1,:),gt(2,:),'.b')
figure
%Error graph in X since row 2 to row 4
plot(Ex(2:4))
hold on
plot(sigmax,'r')
plot(-sigmax,'r')





