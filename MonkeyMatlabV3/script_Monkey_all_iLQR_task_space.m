
% init dynamics
script_Monkey_all_basic;


%% %%%%%%%%%%%%%%%%%%%%% set up control goal
% start and end locations
xz_start = [-0.39;
                  -0.02];
              
xz_end = [0.39;
            0.0];

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

%% generate trajectory for PFL
% total time should be 1.2, then 1.2/200=0.006
step = 300;
T= 0.66;
dt = T/step;

thetastart = [init_angle_left_hand; init_angle_left_shoulder; init_angle_right_shoulder];
thetaend = [end_angle_left_hand; end_angle_left_shoulder; end_angle_right_shoulder];
x0 = [thetastart;[0;0;0]];
xT = [thetaend;[0;0.0;0.0]];

% % pfl joint trajectory
% pfl_n = 3;
Tf = T;
% pfl_N= 2000;
% method = 3 ;
% traj = JointTrajectory(thetastart, thetaend, Tf, pfl_N, method);
% thetamat = traj;
% dthetamat = zeros(pfl_N, pfl_n);
% ddthetamat = zeros(pfl_N, pfl_n);
% pfl_dt = Tf / (pfl_N - 1);
% for i = 1: pfl_N - 1
%   dthetamat(i + 1, :) = (thetamat(i + 1, :) - thetamat(i, :)) / pfl_dt;
%   ddthetamat(i + 1, :) = (dthetamat(i + 1, :) - dthetamat(i, :)) / pfl_dt;
% end
% 
% sim('Monkey_all_pfl','StopTime', num2str(T));

%% load init control from pfl
load('test045_to_045.mat');
t = 0:dt:T;
init_u = new_u_list;

% figure(1)
% plot(t, init_u(1,:),t, init_u(2,:))
% title('PFL Torque');

Q = 0*eye(6);
R = 0.4*eye(2);
Qf = 2000*eye(6);
Qf(4,4) = 0;
% Qf(5,5) = 0;
% Qf(6,6) = 0;

% [pfl_x_list, J] = Monkey_all_complete_run(x0, xT, init_u, dt, step, Q, R, Qf, g, Ftip, Mlist, Glist, Slist);
% dd = sprintf('Cost for PFL strategy %6.5f\n',J);
% disp(dd);

out_u_time = t;
out_u_data = [init_u ];

sim('Monkey_all_use_out_u','StopTime', num2str(T));

%% iLQR step
new_u_list = init_u;
% new_x_list = pfl_x_list;
% TODO add random noise for initial u
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










