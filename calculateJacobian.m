function J = calculateJacobian(config, Tb0, M0e, Blist)
    theta = config(4:8)';   
    % Arm Jacobian
    Jarm = JacobianBody(Blist,theta);
    
    % Base Jacobian
    r = 0.0475; 
    l = 0.235; 
    w = 0.15;
    F = (r/4) * [-1/(l+w),  1/(l+w),  1/(l+w), -1/(l+w);
                  1,         1,        1,        1;
                 -1,         1,       -1,        1];
    
    T0e = FKinBody(M0e, Blist, theta); %Using Forward Kinematics
    
    Teb = inv(Tb0 * T0e);
    Ad_Teb = Adjoint(Teb);
    
    F6 = [zeros(2,4); F; zeros(1,4)]; % Expand F to 6x4 for dimensionality 
    Jbase = Ad_Teb * F6 ;
    
    % Complete Jacobian
    J = [Jbase, Jarm];
end

