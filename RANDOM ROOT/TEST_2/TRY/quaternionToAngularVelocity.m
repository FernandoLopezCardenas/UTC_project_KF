% Función para calcular la velocidad angular desde un quaternion y su derivada
function omega = quaternionToAngularVelocity(q, dq)
    % q: quaternion [w, x, y, z]
    % dq: derivada del quaternion [dw, dx, dy, dz]

    % Conjugado del quaternion
    q_conj = [q(1), -q(2), -q(3), -q(4)];

    % Producto de cuaterniones (2 * dq * q_conj)
    omega_q = 2 * quaternionProduct(dq, q_conj);

    % Extraer las partes vectoriales (velocidad angular)
    omega = omega_q(2:4);  % [omega_x, omega_y, omega_z]
end
