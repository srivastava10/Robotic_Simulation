function Tse = getCurrentEndEffectorConfiguration(config, Tb0, M0e, Blist)
    % To calculate the current end-effector configuration in space frame
    
    % Extract configurations
    phi = config(1); 
    x = config(2); 
    y = config(3);
    theta = config(4:8)';
    
    % {b} frame transformation matrix fixed on chassis
    Tsb = [cos(phi), -sin(phi), 0, x;
           sin(phi),  cos(phi), 0, y;
           0,         0,        1, 0.0963;
           0,         0,        0, 1];
    
    T0e = FKinBody(M0e,Blist,theta);
    
    % Complete transformation
    Tse = Tsb * Tb0 * T0e;
end