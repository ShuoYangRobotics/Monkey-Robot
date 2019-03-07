function x_list = monkey_state_run(u,dt)
init_angle_left_hand = -45/180*pi;
init_angle_left_shoulder = 45/180*pi;
init_angle_right_shoulder = 45/180*pi;

thetastart = [init_angle_left_hand; init_angle_left_shoulder; init_angle_right_shoulder];

 % this does not matter?

% u is a single step*2 x 1 vector
step = size(u,2)/2;
x0 = [thetastart;[0;0;0]];
x_list = zeros(6,step);
x_list(:,1) = x0;
for i = 2:step 
    x_list(:,i) = x_list(:,i-1) + dt*monkey_dyn_func(x_list(:,i-1) , [0; u((i-2)*2+1);u((i-2)*2+2)]) ;
end
    

end
