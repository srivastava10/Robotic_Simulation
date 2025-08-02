function [next_config] = NextState(config,u,theta_dot,timestep,ControlLimit)
%   Maintaining the Control Limit over velocities
    for i=1:4
        if u(i)>ControlLimit
            u(i)=ControlLimit;
        end
        if u(i)<-ControlLimit
            u(i)=-ControlLimit;
        end
    end
    for i=1:5
        if theta_dot(i)>ControlLimit
            theta_dot(i)=ControlLimit;
        end
        if theta_dot(i)<-ControlLimit
            theta_dot(i)=-ControlLimit;
        end
    end

%   EulerSteps
    arm_angles = config(4:8);
    wheel_angles = config(9:12);

    new_arm_angles = arm_angles+theta_dot*timestep;
    new_wheel_angles = wheel_angles+u*timestep;

%   Odometry (Pg.547)
    delta_theta = new_wheel_angles-wheel_angles;
    r = 0.0475;
    l = 0.235;
    w = 0.15;
    Vb = (r/4)*[-1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w); 1 1 1 1; -1 1 -1 1]*delta_theta';
    wbz = Vb(1);
    vbx = Vb(2);
    vby = Vb(3);
    if wbz == 0
        delta_qb = Vb;
    else
        delta_qb = [wbz; vbx*sin(wbz)+vby*(cos(wbz)-1)/wbz; vby*sin(wbz)+vbx*(1-cos(wbz))/wbz];
    end
    phi_k = config(1);
    delta_q = [1 0 0; 0 cos(phi_k) -sin(phi_k); 0 sin(phi_k) cos(phi_k)]*delta_qb;

    q_k = [config(1);config(2);config(3)];

    new_q_k = q_k+delta_q;

    next_config=[new_q_k' new_arm_angles new_wheel_angles];

end

