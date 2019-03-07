function J = monkey_cost_func_direct_collocation(state_u) 
    % u is 1xstep*2
    step = 100;
    u = state_u(1, 6*step+1:end);
    J = u*u';
   
%     end_angle_left_hand = 45/180*pi;
%     end_angle_left_shoulder = -45/180*pi;
%     end_angle_right_shoulder = (45+270)/180*pi;
% 
%     thetaend = [end_angle_left_hand; end_angle_left_shoulder; end_angle_right_shoulder];
%     x_list = monkey_state_run(u);
% 
%     J = u*u' + 9000000* norm(x_list(1:3)-thetaend);
end