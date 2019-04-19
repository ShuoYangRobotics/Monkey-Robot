
% init dynamics
script_Monkey_all_basic;


%% %%%%%%%%%%%%%%%%%%%%% set up control goal
% start and end locations

dist1 = -0.33;
release_height = -0.00;

dist2 = 0.36;
reach_height = 0.0;

xz_start = [dist1;
            release_height];
              
xz_end = [dist2;
          reach_height];
      
save_file_name = strcat('test',num2str(abs(dist1)),'_',num2str(abs(dist2)),'.mat');
save_file_name(save_file_name=='.')=[];

% kinematics
L = (309.8)*10^-3;

% IK
        
r1 = norm(xz_start);
r2 = norm(xz_end);
offset_body_angle = 0/180*pi;

init_angle_left_hand = -asin(r1/2/L)+atan(xz_start(2)/xz_start(1));
init_angle_left_shoulder = 0/180*pi - offset_body_angle;
init_angle_right_shoulder = -2*asin(r1/2/L) + offset_body_angle;

end_angle_left_hand = asin(r2/2/L)+atan(xz_end(2)/xz_end(1));
end_angle_left_shoulder = 0/180*pi + offset_body_angle;
end_angle_right_shoulder = 2*asin(r2/2/L)-2*pi - offset_body_angle;        
%%%%%%%%%%%%%%%%%%%%%%%

%% trajectory parameters
step = 300;
T= 0.66;
dt = T/step;


Q = 0*eye(6);
R = 0.3*eye(2);
Qf = 6400*eye(6);
Qf(4,4) = 0;
Qf(5,5) = 0;
Qf(6,6) = 0;

thetastart = [init_angle_left_hand; init_angle_left_shoulder; init_angle_right_shoulder];
thetaend = [end_angle_left_hand; end_angle_left_shoulder; end_angle_right_shoulder];
x0 = [thetastart;[0;0;0]];
xT = [thetaend;[0;0.0;0.0]];

%% load init control from file
load('test035_033mat.mat');
% this file contains tgt_t tgt_u tgt_state

t = 0:dt:T;
% inteprete trajectory
init_u = interp1(tgt_time', tgt_u',t);
init_u = init_u';

init_x_list = interp1(tgt_time', tgt_state',t);
init_x_list = init_x_list';

out_u_time = t;
out_u_data = [init_u ];

sim('Monkey_all_use_out_u','StopTime', num2str(T));

%% iLQR step
new_u_list = init_u;
new_x_list = init_x_list;
x0 = x0 + 1e-5*randn(size(x0));

total_iter = 70;
lamb = 6.0; % regularization parameter
sim_new_trajectory  = 1;

for i = 1:total_iter
%     try
        [new_x_list,new_u_list, J, lamb, sim_new_trajectory, converge] = ...
            Monkey_all_iLQR_iteration(x0, xT, new_u_list, new_x_list, dt, step, Q, R, Qf, lamb, sim_new_trajectory, g, Ftip, Mlist, Glist, Slist); 
        dd = sprintf('Iteration %d, cost %6.5f\n', i, J);
        disp(dd);
        
%         out_u_time = t;
%         out_u_data = new_u_list;
%         sim('Monkey_all_use_out_u');
        if (converge == 1)
            break;
        end
%     catch ME
%         disp(ME.identifier);
%         disp('exit iteration')
%         break;
%     end
end

figure(3)
plot(t, new_u_list(1,:) ,t, new_u_list(2,:) )
title('iLQR Torque');

out_u_time = t;
out_u_data = new_u_list;
sim('Monkey_all_use_out_u','StopTime', num2str(T));

% save control s
tgt_time = t;
tgt_u = new_u_list;
tgt_state = new_x_list;

save(save_file_name,'tgt_time', 'tgt_u',  'tgt_state');










