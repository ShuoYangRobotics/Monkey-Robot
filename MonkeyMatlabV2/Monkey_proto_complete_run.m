function [x_list, J] = Monkey_proto_complete_run(x0, xT, u, dt, step, Q, R, Qf, g, Ftip, Mlist, Glist, Slist)

% u is a single 2 x step vector
x_list = zeros(6,step+1);
J = 0;
x_list(:,1) = x0;
for i = 1:step 
    tmp_state = x_list(:,i);
    x_list(:,i+1) = x_list(:,i) + dt*Monkey_proto_dyn_func(tmp_state, [0;u(:,i)], g, Ftip, Mlist, Glist, Slist);
    J = J + ( x_list(:,i)'*Q*x_list(:,i) + u(:,i)'*R*u(:,i) )*dt;  % <-- better integration?
end
J = J + (x_list(:,end) - xT)'*Qf*(x_list(:,end) - xT);

end
