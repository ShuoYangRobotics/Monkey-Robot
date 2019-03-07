function [new_x_list,new_u_list] = monkey_iLQR_iteration(x0_list, xT, u_0, step, dt)
    % define cost
    Q = 0*eye(6);
    R = 1*eye(2);
    Qf = 8270*eye(6);
    
    % forward pass with new_u
    % calculate cost for reference
    J = 0;
    new_x_list = zeros(6,step+1);
    new_x_list(:,1) = x0_list(:,1);
    for i = 1:step
        tmp_state = new_x_list(:,i);
        x_dot = monkey_dyn_func(tmp_state, [0;u_0(:,i)]);
        new_x_list(:,i+1) = new_x_list(:,i) + dt*x_dot;
        
        J = J + new_x_list(:,i)'*Q*new_x_list(:,i) + u_0(:,i)'*R*u_0(:,i);
    end
%     figure(3)
%     plot(new_x_list(1,:),new_x_list(3,:))
%     title('mid');
    J = J + (new_x_list(:,end) - xT)'*Qf*(new_x_list(:,end) - xT);
    J
    V_x = Qf*new_x_list(:,end) - Qf*xT;
    V_xx = Qf;
    
    ff_k = zeros(2,1,step);
    fb_K = zeros(2,6,step);
    
    for i = step-1:-1:1
        % calculate of these terms can be parallelled
        J_A_lower = A_func(x0_list(1,i),x0_list(2,i),x0_list(3,i),...
                           x0_list(4,i),x0_list(5,i),x0_list(6,i),...
                           u_0(1,i),u_0(2,i));
        J_A_full = [zeros(3,3) eye(3); J_A_lower];              
        A = eye(6) + dt*J_A_full;
        
        B = dt*B_func(x0_list(2,i),x0_list(3,i));
        
        Q_x = Q * x0_list(:,i) + A' * V_x;
        Q_u = R * u_0(:,i) + B'*V_x;
        
        Q_xx = Q + A'*V_xx*A;
        Q_ux = B'*V_xx*A;
        Q_uu = R + B'*V_xx*B;
        [evecs,evals] = eig(Q_uu);
        evals(evals < 0) = 0.0;
        evals = evals + 22;
        Q_uu_inv = evecs * 1./evals * evecs';
        

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
    
    new_u_list = zeros(2,step);
    
    for i = 1:step
        new_u_list(:,i) = u_0(:,i) + 0.2*ff_k(:,:,i) + fb_K(:,:,i)*(new_x_list(:,i) - x0_list(:,i));
    end
    
end