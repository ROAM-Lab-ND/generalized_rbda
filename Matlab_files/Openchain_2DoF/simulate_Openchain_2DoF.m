close all
%%
simulate_openchain_2dof

function simulate_openchain_2dof()

    %% Define fixed parameters
    m1 = 1.;
    m2 = 1.;
    l1 = 0.7;
    l2 = .6;
    c1 = .5;
    c2 = .25;
    I1 = 0.05;
    I2 = 0.05;
    I1rot = 0.00003;
    I2rot = 0.00003;
    N1 = 6;
    N2 = 5;
    g  = 9.81;    
    p   = [l1; l2 ; c1; c2; m1; m2; I1; I2; I1rot; I2rot; N1; N2; g];

    %% Q2.1 Perform simulation
    dt = 0.0001;
    tf = 10;
    num_step = floor(tf/dt);
    tspan = linspace(0, tf, num_step); 
    
    z0 = [3 0 0 0]';
    z_out = zeros(4,num_step);
    z_out(:,1) = z0;
    for i=1:num_step-1
        dz = dynamics(z_out(:,i), p);
        z_out(3:4,i+1) = z_out(3:4,i) + dz(3:4)*dt;
        z_out(1:2,i+1) = z_out(1:2,i) + z_out(3:4,i+1)*dt + 0.5*dz(3:4)*dt*dt;
        %z_out(1:2,i+1) = z_out(1:2,i) + z_out(3:4,i+1)*dt;
    end
    figure(1)
    clf
    %% Q2.2 Plot Angles
    plot(tspan,z_out(1:2,:))
    xlabel('Time (s)')
    ylabel('Angle (rad)');
    legend({'\theta_1','\theta_2'},'FontSize',16);
    
    figure(2)
    clf
    %% Q2.3 Plot Energy
    E = E_openchain_2dof(z_out,p);
    plot(tspan,E);
    xlabel('Time (s)');
    ylabel('Total Energy (J)');
    
    figure(3)
    clf
    %% Q2.1 Animation
    h_pend = plot([0,0],'LineWidth',4);
    skip_frame = 500;
    axis([-2 2 -2 2])
    xlabel('x (m)');
    ylabel('y (m)')
    h_title = title('t=0.0s');
    for i=1:num_step
        if mod(i, skip_frame)
            continue
        end
        t = tspan(i);
        z = z_out(:,i);
        keypoints = keypoints_openchain_2dof(z,p);        
        h_pend.XData = [ keypoints(1,:) ];
        h_pend.YData = [ keypoints(2,:) ];
        pause(.03);
        h_title.String = sprintf('t=%.2fs',t);
    end
end

function dz = dynamics(z,p)
        A = A_openchain_2dof(z,p);        
        u = [0 0]';
        b = b_openchain_2dof(z,u,p);
        qdd = A\b;
        dz = 0*z;
        dz(1:2) = z(3:4);
        dz(3:4) = qdd;       
end
    
