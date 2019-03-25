function [ c , ceq ] = monkey_shooting_constraint_func(u)

init_angle_left_hand = -45/180*pi;
init_angle_left_shoulder = 45/180*pi;
init_angle_right_shoulder = 45/180*pi;

end_angle_left_hand = 45/180*pi;
end_angle_left_shoulder = -45/180*pi;
end_angle_right_shoulder = (45+270)/180*pi;

thetastart = [init_angle_left_hand; init_angle_left_shoulder; init_angle_right_shoulder];
thetaend = [end_angle_left_hand; end_angle_left_shoulder; end_angle_right_shoulder];

dt = 0.012; % this does not matter?

% u is a single step*2 x 1 vector
step = size(u,2)/2;
x = [thetastart;[0;0;0]];
xT = [thetaend;[0;0;0]];

for i = 1:step 
    x = x + dt*monkey_dyn_func(x , [0; u(i);u(i+1)]) ;
end

% ceq = abs(x-xT);
% c = [];
% 02-25 21:21 try some different constraint
ceq = abs(x(1:3)-xT(1:3));
c = [-xT(4);-xT(6)];


end