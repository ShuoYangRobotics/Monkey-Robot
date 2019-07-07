function [new_x0, new_x_list,new_u_list,J,lamb, sim_new_trajectory, converge] = Monkey_all_iLQR_iteration(x0, xT, new_u_list, new_x_list,  dt, step, Q, R, Qf, lamb, sim_new_trajectory, g, Ftip, Mlist, Glist, Slist)
    lamb_factor = 2.5;
    lamb_max = 10000;
    % store fx fu lx lu
    persistent fx fu lx lu lfinal old_J;
    if isempty(fx)
        fx = zeros(6,6,step);
    end
    if isempty(fu)
        fu = zeros(6,2,step);
    end
    if isempty(lx)
        lx = zeros(6,1,step);
    end
    if isempty(lu)
        lu = zeros(2,1,step);
    end
    if isempty(lfinal)
        lfinal = zeros(6,1);
        
    end
    if isempty(old_J)
        old_J = 9e9;
    end
    
    % forward pass with existing u_list
    if sim_new_trajectory == 1
        dd = sprintf('simulate with new x0 %6.5f %6.5f %6.5f\n', x0(1),x0(2),x0(3));
        disp(dd);
        [sim_x_list, old_J] = Monkey_all_complete_run(x0, xT, new_u_list, dt, step, Q, R, Qf, g, Ftip, Mlist, Glist, Slist);
        new_x_list = sim_x_list;
        
        for i = 1:step
            J_A_lower = A_func(new_x_list(1,i),new_x_list(2,i),new_x_list(3,i),...
                   new_x_list(4,i),new_x_list(5,i),new_x_list(6,i),...
                   new_u_list(1,i),new_u_list(2,i));
            J_A_full = [zeros(3,3) eye(3); J_A_lower];              
            A = eye(6) + dt*J_A_full;
            fx(:,:,i) = A;

            B = dt*B_func(new_x_list(2,i),new_x_list(3,i));
            fu(:,:,i) = B;
            
            lx(:,:,i) = Q * new_x_list(:,i);
            lu(:,:,i) = R * new_u_list(:,i);
            lfinal = Qf*new_x_list(:,end) - Qf*xT;
        end
        sim_new_trajectory = 0;
    end
    
    % optimize 
    V = (new_x_list(:,end) - xT)'*Qf*(new_x_list(:,end) - xT);
    V_x = lfinal;
    V_xx = Qf;
    
    ff_k = zeros(2,1,step);
    fb_K = zeros(2,6,step);
    
    for i = step-1:-1:1         

        % following is udp step
        [S,d] = chol(blkdiag(V_xx^-1, R^-1), 'lower');
        if d
            disp('cannot take chol of blkdiag(V_xx^-1, R^-1)')
            return;
        end
        scale = 0.01;
        S = scale*S;
        Sig = [S -S];
        for j = 1:(2*(6+2))
            Sig(:,j) = Sig(:,j) + [new_x_list(:,i+1); new_u_list(:,i)];
        end
        %Project vx(i+1) onto sigma points
        pg = zeros(2*(6+2),1);
        for j = 1:(6+2)
            pg(j) = V_x'*S(1:6,j);
            pg(j+6+2) = -pg(j);
        end
        %Propagate sigma points through backwards dynamics
        for j = 1:(2*(6+2))
            Sig(1:6,j) = rkstep_b(Sig(:,j),6,dt, g, Ftip, Mlist, Glist, Slist);
        end
        
        %Calculate [Qu; Qx] from sigma points
        D = zeros(6+2,6+2);
        df = zeros(6+2,1);
        for j = 1:(6+2)
            D(j,:) = (Sig(:,j)-Sig(:,6+2+j))';
            df(j) = pg(j) - pg(6+2+j);
        end
        QxQu = D\df;
        Q_x = QxQu(1:6) + lx(:,:,i); %add on one-step cost
        Q_u = QxQu((6+1):end) + lu(:,:,i); %add on one-step cost
        
        %Calculate Hessian w.r.t. [x_k; u_k] from sigma points
        mu = zeros(6+2,1);
        for j = 1:(2*(6+2))
            mu = mu + (1/(2*(6+2)))*Sig(:,j);
        end
        M = zeros(6+2);
        for j = 1:(2*(6+2))
            M = M + (0.5/scale^2)*(Sig(:,j)-mu)*(Sig(:,j)-mu)';
        end
        H = M^-1;
        H(1:6,1:6) = H(1:6,1:6) + Q; %add in one-step state cost for this timestep

        Q_xx = H(1:6,1:6);
        Q_uu = H(6+1:end,6+1:end);
        Q_ux = H(6+1:end,1:6);
        % udp step ends
        

        % standard regularization
        [evecs,evals] = eig(Q_uu);
        evals(evals < 0) = 0.0;
        Q_uu_inv = evecs * diag(1./(diag(evals)+lamb*ones(2,1))) * evecs';
        
        

        ff_k(:,:,i) = - Q_uu_inv*Q_u;
        fb_K(:,:,i) = - Q_uu_inv*Q_ux;
        
        delta_V = -0.5*Q_u'* Q_uu_inv * Q_u;
        V = V - delta_V;

        V_x = Q_x + fb_K(:,:,i)'*Q_uu*ff_k(:,:,i) +fb_K(:,:,i)'*Q_u + Q_ux'*ff_k(:,:,i);
        V_xx = Q_xx + fb_K(:,:,i)'*Q_uu*fb_K(:,:,i)+fb_K(:,:,i)'*Q_ux + Q_ux'*fb_K(:,:,i);
        V_xx  = .5*(V_xx + V_xx');
    end
    
    update_u_list = zeros(2,step+1);
    
    update_x = x0;
    
    for i = 1:step
        update_u_list(:,i) = new_u_list(:,i) + 0.1*ff_k(:,:,i) + fb_K(:,:,i)*(update_x - new_x_list(:,i));
        tmp_state = update_x;
        update_x = update_x + dt*Monkey_all_dyn_func(tmp_state, [0;update_u_list(:,i)], g, Ftip, Mlist, Glist, Slist);
    end
    new_x0 = x0;
    %evaluate the update trajectory
    [update_x_list, new_J] = Monkey_all_complete_run(new_x0, xT, update_u_list, dt, step, Q, R, Qf, g, Ftip, Mlist, Glist, Slist);
    
    
    V
    
    % Levenberg-Marquardt heuristic
    if (new_J < old_J)
        J = new_J;
        lamb = lamb / lamb_factor;

        new_x_list = update_x_list;
        new_u_list = update_u_list;
        
        sim_new_trajectory = 1;
        converge = 0;
        if (old_J - new_J)/new_J < 0.05  %
            disp('converge')
            converge = 1;
        end
    else
        J = old_J;
        converge = 0;
        lamb = lamb * lamb_factor;
        if lamb > lamb_max
            dd = sprintf('lambda is %6.5f, bad!\n',lamb);
            disp(dd)
            converge = 1;
        end
        
    end
    
end


function x0 = rkstep_b(x1u0,Nx,dt, g, Ftip, Mlist, Glist, Slist)
    %Backwards 4th order Runge-Kutta step from x_{k+1} to x_k
    xdot1 = Monkey_all_dyn_func(x1u0(1:Nx),[0;x1u0((Nx+1):end)], g, Ftip, Mlist, Glist, Slist);
    xdot2 = Monkey_all_dyn_func(x1u0(1:Nx)-.5*dt*xdot1,[0;x1u0((Nx+1):end)], g, Ftip, Mlist, Glist, Slist);
    xdot3 = Monkey_all_dyn_func(x1u0(1:Nx)-.5*dt*xdot2,[0;x1u0((Nx+1):end)], g, Ftip, Mlist, Glist, Slist);
    xdot4 = Monkey_all_dyn_func(x1u0(1:Nx)-dt*xdot3,[0;x1u0((Nx+1):end)], g, Ftip, Mlist, Glist, Slist);
    
    x0 = x1u0(1:Nx) - (dt/6)*(xdot1 + 2*xdot2 + 2*xdot3 + xdot4);
end
    