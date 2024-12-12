% Función para procesar la base de datos de quaterniones
function angularVelocities = processQuaternionDatabase(quaternions, dquaternions)
    % quaternions: matriz Nx4 de quaterniones [w, x, y, z]
    % quaternionsDot: matriz Nx4 de derivadas de quaterniones [dw, dx, dy, dz]
    % angularVelocities: matriz Nx3 de velocidades angulares [omega_x, omega_y, omega_z]
    
    % Número de muestras
    n = size(quaternions, 1);
    
    % Inicializar matriz de velocidades angulares
    angularVelocities = zeros(n, 3);
    
    % Procesar cada muestra
    for i = 1:n
        % Obtener el quaternion y su derivada
        q = quaternions(i, :);
        dq = dquaternions(i, :);
        
        % Calcular la velocidad angular
        angularVelocities(i, :) = quaternionToAngularVelocity(q, dq);
    end
end
