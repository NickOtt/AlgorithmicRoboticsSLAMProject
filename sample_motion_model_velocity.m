function xt = sample_motion_model_velocity(ut,xtprior,alpha)
    dt = 1;
    vt = ut(1) + sample_normal_distrib(alpha(1)*ut(1)^2 + alpha(2)*ut(2)^2);
    wt = ut(2) + sample_normal_distrib(alpha(3)*ut(1)^2 + alpha(4)*ut(2)^2);
    gamma = sample_normal_distrib(alpha(5)*ut(1)^2 + alpha(6)*ut(2)^2);
    xt=[0;0;0];
    xt(1) = xtprior(1) - vt/wt*sin(xtprior(3)) + vt/wt*sin(xtprior(3)+wt*dt);
    xt(2) = xtprior(2) + vt/wt*cos(xtprior(3)) - vt/wt*cos(xtprior(3)+wt*dt);
    xt(3) = xtprior(3) + wt*dt + gamma*dt;
end

