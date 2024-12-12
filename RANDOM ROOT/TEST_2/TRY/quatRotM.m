function Q = quatRotM(qt)
    q0 = qt(1);      
    qv = qt(2:4);    
    Q = (2*q0^2-1)*eye(3)+2*(qv*qv')+2*q0*PCO(qv);
end

