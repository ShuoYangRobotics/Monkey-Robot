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
    
    move_x =[];
    new_x0 = [];
    
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
        
        delta_V = -0.5*Q_u'* Q_uu_inv * Q_u;
        V = V - delta_V;
        % Synthesis and Stabilization of Complex Behaviors through Online Trajectory Optimization
        % mentioned two diffrerent implementation for this (7 and 11)
%         V_x = Q_x - fb_K(:,:,i)'*Q_uu*ff_k(:,:,i);
%         V_xx = Q_xx - fb_K(:,:,i)'*Q_uu*fb_K(:,:,i);
        V_x = Q_x + fb_K(:,:,i)'*Q_uu*ff_k(:,:,i) +fb_K(:,:,i)'*Q_u + Q_ux'*ff_k(:,:,i);
        V_xx = Q_xx + fb_K(:,:,i)'*Q_uu*fb_K(:,:,i)+fb_K(:,:,i)'*Q_ux + Q_ux'*fb_K(:,:,i);
        V_xx  = .5*(V_xx + V_xx');
        % makes no difference
        if (i == 1)

%             delta_V = -0.5*Q_u'* Q_uu_inv * Q_u;
%             q = delta_V
%             H = V_xx;
%             H=(H+H')/2;
%             f = V_x;
%             V_x
%             [evecs,evals] = eig(H, 'vector');
%                 
%             a = (evals(2) + evals(3))/2;
%             tmp = f'*evecs;
%             b = tmp(2) - tmp(3);
%             c = q;
%             move_x1 = evecs'*(-b + sqrt(b*b-4*a*c))/(2*a)* [0;1;-1;0;0;0];
%             move_x2 = evecs'*(-b - sqrt(b*b-4*a*c))/(2*a)* [0;1;-1;0;0;0];
% 
%             v1 = 0.5*move_x1'*H*move_x1 + f'*move_x1 + q
%             v2 = 0.5*move_x2'*H*move_x2 - f'*move_x2 + q
%             if (abs(v1)<abs(v2))
%                 move_x = evecs*move_x1;
%             else
%                 move_x = evecs*move_x2;
%             end
%             move_x;
%             V
%             V_x
%             Aeq = [1 0  0 0 0 0
%                    0 1 1 0 0 0
%                    0 0  0 1 0 0
%                    0 0  0 0 1 0
%                    0 0  0 0 0 1];
%             beq = [0;0;0;0;0];
%             move_x = quadprog((V_xx+V_xx')/2,V_x,[],[],Aeq,beq)
%             x0;
            new_x0 = x0;
%             V_xx_inv = evecs * diag(1./(diag(evals))) * evecs';
%             evals(2,2)
%             evals(3,3)
%             x = sqrt(-2*q/(evals(2,2)+evals(3,3))) * [0;1;-1;0;0;0];
%             0.5*x'*H*x + f'*x
%             0.5*x'*H*x + f'*x
%             H = V_xx;
%             H=(H+H')/2;
%             f = V_x;
%             Aeq = [1 0  0 0 0 0
%                    0 1 1 0 0 0
%                    0 0  0 1 0 0
%                    0 0  0 0 1 0
%                    0 0  0 0 0 1];
%             beq = [0;0;0;0;0];
%             x = quadprog(H,f,[],[],Aeq,beq)
%             0.5*x'*H*x + f'*x
        end
    end
    
    update_u_list = zeros(2,step+1);
    
    
    
    % 2019-6-25 add new update u
%     B = [1 0 0
%          0 1 0
%          0 0 1
%          0 0 0
%          0 0 0
%          0 0 0];
%     Q_u = B'*V_x;    
%     Q_ux = B'*V_xx;
%     Q_uu = B'*V_xx*B;
%     % standard regularization
%     [evecs,evals] = eig(Q_uu);
%     evals(evals < 0) = 0.0;
%     Q_uu_inv = evecs * diag(1./(diag(evals)+lamb*ones(3,1))) * evecs';
%     ff_x0 = - Q_uu_inv*Q_u;
%     fb_x0 = - Q_uu_inv*Q_ux;
    % project ff_x0 to [0;1;-1]     
%     [evecs,evals] = eig(V_xx);
%     evals(evals < 0) = 0.0;
%     V_xx_inv = evecs * diag(1./(diag(evals)+lamb*ones(6,1))) * evecs';
%     x_0_constraint = [0;1;-1;0;0;0];
%     update_x0 = 0.007*dot(-V_xx_inv*V_x,x_0_constraint)/norm(x_0_constraint)^2*x_0_constraint;
%      
%     dd = sprintf('old x0 %6.5f %6.5f %6.5f\n', x0(1),x0(2),x0(3));
%     disp(dd);
%     new_x0 = x0 + update_x0;
%     dd = sprintf('new x0 %6.5f %6.5f %6.5f\n', new_x0(1),new_x0(2),new_x0(3));
%     disp(dd);

%     new_x0 = x0;
    update_x = new_x0;
    
    for i = 1:step
        update_u_list(:,i) = new_u_list(:,i) + 0.1*ff_k(:,:,i) + fb_K(:,:,i)*(update_x - new_x_list(:,i));
        tmp_state = update_x;
        update_x = update_x + dt*Monkey_all_dyn_func(tmp_state, [0;update_u_list(:,i)], g, Ftip, Mlist, Glist, Slist);
    end
    
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
%             
%             [evecs,evals] = eig(V_xx);
%             V_xx_inv = evecs * diag(1./(diag(evals)+lamb*ones(6,1))) * evecs';
%             x_0_constraint = [0;1;-1;0;0;0];
%             update_x0 = 0.007*dot(-V_xx_inv*V_x,x_0_constraint)/norm(x_0_constraint)^2*x_0_constraint;
% 
% 
% % 
%         V
%         Aeq = [1 0  0 0 0 0
%                0 -1 -1 0 0 0
%                0 0  0 1 0 0
%                0 0  0 0 1 0
%                0 0  0 0 0 1];
%         beq = [0;0;0;0;0];
%         update_x0 = quadprog((V_xx+V_xx')/2,V_x,[],[],Aeq,beq,-0.05*ones(6,1),0.05*ones(6,1))
% %             update_x0 = linprog(-V_x,eye(6),0.1*ones(6,1),Aeq,beq)
%         0.5*update_x0'*(V_xx+V_xx')/2*update_x0 + V_x'*update_x0
% %             0.5*-update_x0'*(V_xx+V_xx')/2*-update_x0 + V_x'*-update_x0
%         new_x0 = x0+update_x0;
%         if norm(update_x0) < 1e-4

            disp('converge')

            converge = 1;
%             end
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


    