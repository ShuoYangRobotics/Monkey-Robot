function qdot = Monkey_proto_dyn_func ( state , action, g, Ftip, Mlist, Glist, Slist) 
% state is q1 q2 q3 q1_dot q2_dot q3_dot
thetalist = state(1:3);
dthetalist = state(4:6);

ddthetalist = ForwardDynamics(thetalist, dthetalist, action, ...
                                   g, Ftip, Mlist, Glist, Slist);
                                   
% qdot =  q1_dot q2_dot q3_dot q1_ddot q2_ddot dq3_dot                                 
qdot = [ state(4);
         state(5);
         state(6);
         ddthetalist];
end