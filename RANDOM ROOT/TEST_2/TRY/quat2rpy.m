function rpy = quat2rpy(qt)
    Q = quatRotM(qt);
    rpy = zeros(3,1);

    rpy(2) = atan(-Q(3,1)/sqrt(Q(1,1)^2+Q(2,1)^2));

    rpy(1) = atan2(Q(3,2)/cos(rpy(2)),Q(3,3)/cos(rpy(2)));
    
    rpy(3) = atan2(Q(2,1)/cos(rpy(2)),Q(1,1)/cos(rpy(2)));
end

