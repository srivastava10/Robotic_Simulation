function trajectory = ScrewTrajectoryMod(Xstart, Xend, Tf, N, method, gripperState)
% Modified screw trajectory with better interpolation
timegap = Tf / (N - 1);
trajectory = [];

for i = 1:N
    if method == 3
        s = CubicTimeScaling(Tf, timegap * (i - 1));
    else
        s = QuinticTimeScaling(Tf, timegap * (i - 1));
    end
    
    next_SE3 = Xstart * MatrixExp6(MatrixLog6(TransInv(Xstart) * Xend) * s);
    traj = [next_SE3(1) next_SE3(5) next_SE3(9) next_SE3(2) next_SE3(6) next_SE3(10) ...
            next_SE3(3) next_SE3(7) next_SE3(11) next_SE3(13) next_SE3(14) next_SE3(15) gripperState];
    trajectory = [trajectory; traj];
end
end

