function J = monkey_cost_func(u) 
    % u is 1xstep*2
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