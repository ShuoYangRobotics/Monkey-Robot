function J = wheel_cost_func(u) 
    dt = 1.2/300;
    radius = 0.25;  % radius of wheel
    m = 4.025;

    xT = [2*sqrt(2);0];
    step = 300;
    x = [0;0];

    for i = 1:step 
        x = x + dt * ([0 1;0 0]*x + [0;1/m/radius]*u(i));
    end
    % u is 1xstep*2
    J = (x-xT)'*(x-xT);
   
%     end_angle_left_hand = 45/180*pi;
%     end_angle_left_shoulder = -45/180*pi;
%     end_angle_right_shoulder = (45+270)/180*pi;
% 
%     thetaend = [end_angle_left_hand; end_angle_left_shoulder; end_angle_right_shoulder];
%     x_list = monkey_state_run(u);
% 
%     J = u*u' + 9000000* norm(x_list(1:3)-thetaend);
end