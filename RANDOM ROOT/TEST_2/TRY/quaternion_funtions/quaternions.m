clear all
close all
clc

%%%%%%%%%%%%%%%%%%%%%%% Registration data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Leer todo el archivo CSV
data = readmatrix('all_logs.csv');

% Extract measured data from X,Y,Theta
q_x = data(:,11);                % Using X as X (AF ins Excel)
q_y = data(:,12);               % Using Z as Y (AH ins Excel)
q_z = data(:,13);                 % Using θ (Y ins Excel)
q_w = data(:,10);                 % Using δ as Steering (K ins Excel)

%%%%%%%%%%%%%%%%%%%%%%%% Mi referencia es el Marconi_computed_path NO ESTE

% Filter non-NaN entries measured data from X,Y,Theta
q_x = q_x(~isnan(q_x));
q_y = q_y(~isnan(q_y));
q_z = q_z(~isnan(q_z));
q_w = q_w(~isnan(q_w));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% quaternions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1: length(data)
    q = [q_w(i) q_x(i) q_y(i) q_z(i)];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    quaternion = quat2eul(q);
    quat(i,:) = rad2deg(quaternion);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    rpy = quat2rpy(q)
    quat2(i,:) = rad2deg(quaternion);

end