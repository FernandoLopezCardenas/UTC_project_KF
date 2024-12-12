function quat = quaternions(q)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%% quaternions angles %%%%%%%%%%%%%%%%%%%%%%%%%%%
    for i = 1: length(q)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        rpy = quat2rpy(q);
        quat(i,:) = rad2deg(rpy);
        % the variable quat keep the degrees for X(roll) Y(pitch) and Z (yaw)
    end
end