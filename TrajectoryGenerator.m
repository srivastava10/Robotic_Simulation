function [trajectory] = TrajectoryGenerator(Tse_initial,Tsc_initial,Tsc_final,Tce_grasp,Tce_standoff,k)

    %SEGMENT 1:
    
    X_start = Tse_initial;
    X_end = Tsc_initial*Tce_standoff;
    Tf=5; % Segment 1 runs for 5 seconds
    N=Tf*k/0.01; % N=tk/0.01 and k=1
    method = 3; % Cubic Time Scaling
    gripperState = 0;
    segment1 = ScrewTrajectoryMod(X_start,X_end,Tf,N,method,gripperState);

    %SEGMENT 2:
    X_start = X_end;
    X_end = Tsc_initial*Tce_grasp;
    Tf=2; % Segment 2 runs for 2 seconds
    N=Tf*k/0.01; % N=tk/0.01 and k=1
    method = 3; % Cubic Time Scaling
    gripperState = 0;
    segment2 = ScrewTrajectoryMod(X_start,X_end,Tf,N,method,gripperState);

    %SEGMENT 3:
    gripperState = 1;
    Tf=1; % As gripper transitions in about 0.625 seconds
    segment3 = ScrewTrajectoryMod(X_end,X_end,Tf,N,method,gripperState);

    %SEGMENT 4:
    X_start = X_end;
    X_end = Tsc_initial*Tce_standoff;
    Tf=1; % Segment 4 runs for 1 seconds
    N=Tf*k/0.01; % N=tk/0.01 and k=1
    method = 3; % Cubic Time Scaling
    gripperState = 1;
    segment4 = ScrewTrajectoryMod(X_start,X_end,Tf,N,method,gripperState);

    %SEGMENT 5:
    X_start = X_end;
    X_end = Tsc_final*Tce_standoff;
    Tf=5; % Segment 5 runs for 5 seconds
    N=Tf*k/0.01; % N=tk/0.01 and k=1
    method = 3; % Cubic Time Scaling
    gripperState = 1;
    segment5 = ScrewTrajectoryMod(X_start,X_end,Tf,N,method,gripperState);

    %SEGMENT 6:
    X_start = X_end;
    X_end = Tsc_final*Tce_grasp;
    Tf=2; % Segment 6 runs for 2 seconds
    N=Tf*k/0.01; % N=tk/0.01 and k=1
    method = 3; % Cubic Time Scaling
    gripperState = 1;
    segment6 = ScrewTrajectoryMod(X_start,X_end,Tf,N,method,gripperState);
    
    %SEGMENT 7:
    gripperState = 0;
    Tf=1; % As gripper transitions in about 0.625 seconds
    segment7 = ScrewTrajectoryMod(X_end,X_end,Tf,N,method,gripperState);

    %SEGMENT 8:
    X_start = X_end;
    X_end = Tsc_final*Tce_standoff;
    Tf=1; % Segment 8 runs for 1 seconds
    N=Tf*k/0.01; % N=tk/0.01 and k=1
    method = 3; % Cubic Time Scaling
    gripperState = 0;
    segment8 = ScrewTrajectoryMod(X_start,X_end,Tf,N,method,gripperState);

    trajectory = [segment1;segment2;segment3;segment4;segment5;segment6;segment7;segment8];    

end

