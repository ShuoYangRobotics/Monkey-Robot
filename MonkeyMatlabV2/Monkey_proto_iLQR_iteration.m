function [new_x_list,new_u_list,J,lamb, sim_new_trajectory, converge] = Monkey_proto_iLQR_iteration(x0, xT, new_u_list, new_x_list,  dt, step, Q, R, Qf, lamb, sim_new_trajectory, g, Ftip, Mlist, Glist, Slist)
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

        [sim_x_list, old_J] = Monkey_proto_complete_run(x0, xT, new_u_list, dt, step, Q, R, Qf, g, Ftip, Mlist, Glist, Slist);
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
    V_x = lfinal;
    V_xx = Qf;
    
    ff_k = zeros(2,1,step);
    fb_K = zeros(2,6,step);
    
    for i = step:-1:1         
        A = fx(:,:,i);
        B = fu(:,:,i);
        
        Q_x = lx(:,:,i) + A' * V_x;
        Q_u = lu(:,:,i) + B'*V_x;
        
        Q_xx = Q + A'*V_xx*A;
        Q_ux = B'*V_xx*A;
        Q_uu = R + B'*V_xx*B;
        % standard regularization
        [evecs,evals] = eig(Q_uu);
        evals(evals < 0) = 0.0;
        Q_uu_inv = evecs * diag(1./(diag(evals)+lamb*ones(2,1))) * evecs';
        
        
%          - Q_uu_inv*Q_u
%         - Q_uu_inv*Q_ux
        ff_k(:,:,i) = - Q_uu_inv*Q_u;
        fb_K(:,:,i) = - Q_uu_inv*Q_ux;
        
        % Synthesis and Stabilization of Complex Behaviors through Online Trajectory Optimization
        % mentioned two diffrerent implementation for this (7 and 11)
%         V_x = Q_x - fb_K(:,:,i)'*Q_uu*ff_k(:,:,i);
%         V_xx = Q_xx - fb_K(:,:,i)'*Q_uu*fb_K(:,:,i);
        V_x = Q_x + fb_K(:,:,i)'*Q_uu*ff_k(:,:,i) +fb_K(:,:,i)'*Q_u + Q_ux'*ff_k(:,:,i);
        V_xx = Q_xx + fb_K(:,:,i)'*Q_uu*fb_K(:,:,i)+fb_K(:,:,i)'*Q_ux + Q_ux'*fb_K(:,:,i);
        % makes no difference
    end
    
    update_u_list = zeros(2,step+1);
    update_x = x0;
    
    for i = 1:step
        update_u_list(:,i) = new_u_list(:,i) + 0.1*ff_k(:,:,i) + fb_K(:,:,i)*(update_x - new_x_list(:,i));
        tmp_state = update_x;
        update_x = update_x + dt*Monkey_proto_dyn_func(tmp_state, [0;update_u_list(:,i)], g, Ftip, Mlist, Glist, Slist);
    end
    
    %evaluate the update trajectory
    [update_x_list, new_J] = Monkey_proto_complete_run(x0, xT, update_u_list, dt, step, Q, R, Qf, g, Ftip, Mlist, Glist, Slist);
    
    % Levenberg-Marquardt heuristic
    if (new_J < old_J)
        J = new_J;
        lamb = lamb / lamb_factor;

        new_x_list = update_x_list;
        new_u_list = update_u_list;
        
        sim_new_trajectory = 1;
        converge = 0;
        if (old_J - new_J)/new_J < 0.03  %
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