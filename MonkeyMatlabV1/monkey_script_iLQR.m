% this script accompanies monkey_use_out_u, it use iLQR method to
% calculate a trajectory out_u. and then monkey_use_out_u will use this
% trajectory to control it
% 2019-03-05 ideally, this method should generate a trajectory that is
% identical to the direct collocation method
% 2019-03-06 it does not give very good control strategy...
% the reason may be line search and regularization
% 2019-03-07 refactor code, implement regularization
% good performance!


%%%%%%%%%%%%%%%%%%%%%%% set up control goal
init_angle_left_hand = -45/180*pi;
init_angle_left_shoulder = 0/180*pi;
init_angle_right_shoulder = 90/180*pi;

end_angle_left_hand = 60/180*pi;
end_angle_left_shoulder = 0/180*pi;
end_angle_right_shoulder = (90+90+120)/180*pi;
%%%%%%%%%%%%%%%%%%%%%%%


monkey_script_basic;

global g  Ftip  Mlist  Glist  Slist
% calculate dynamic model
N=3;
g = [0; 0; -9.8];
Ftip = zeros(6, 1); 

% robot structure
% be careful about the position of these frames 
M1 = [eye(3) [0; 0; -arm_length/2]; [0 0 0 1]];
M2 = [eye(3) [0; -body_depth/2; -arm_length-body_length/2]; [0 0 0 1]];
M3 = [eye(3) [0; -body_depth; -arm_length/2]; [0 0 0 1]];
M4 = [eye(3) [0; -body_depth; 0]; [0 0 0 1]];

M01 = M1;
M12 = inv(M1)*M2;
M23 = inv(M2)*M3;
M34 = inv(M3)*M4;

I1 = [arm_Ixx       0       0;
            0 arm_Iyy       0;
            0       0 arm_Izz ];
        
G1 = [I1 zeros(3,3);
      zeros(3,3) diag([arm_mass, arm_mass, arm_mass])];
  
I2 = [body_Ixx        0        0;
             0 body_Iyy        0;
             0        0 body_Izz ];
G2 = [I2 zeros(3,3);
      zeros(3,3) diag([body_mass, body_mass, body_mass])];
  
I3 = [arm_Ixx       0       0;
            0 arm_Iyy       0;
            0       0 arm_Izz ];
G3 = [I3 zeros(3,3);
      zeros(3,3) diag([arm_mass, arm_mass, arm_mass])];

Glist = cat(3, G1, G2, G3);
Mlist = cat(3, M01, M12, M23, M34); 

w1 = [0;-1;0];
pq1 = [0;0;0];
w2 = [0;-1;0];
pq2 = [0;0;-arm_length];
w3 = [0;-1;0];
pq3 = [0;0;-arm_length];
Slist = [[w1;-cross(w1,pq1)] [w2;-cross(w2,pq2)] [w3;-cross(w3,pq3)]];


% M = MassMatrix(thetalist, Mlist, Glist, Slist);
% grav = GravityForces(thetalist, g, Mlist, Glist, Slist);
% c = VelQuadraticForces(thetalist, dthetalist, Mlist, Glist, Slist);


% put all parameters here
% total time should be 1.2, then 1.2/200=0.006
step = 300;
T= 1.2;
dt = T/step;

Q = 0*eye(6);
R = 0.4*eye(2);
Qf = 2000*eye(6);
Qf(4,4) = 0;
% Qf(5,5) = 0;
% Qf(6,6) = 0;



thetastart = [init_angle_left_hand; init_angle_left_shoulder; init_angle_right_shoulder];
thetaend = [end_angle_left_hand; end_angle_left_shoulder; end_angle_right_shoulder];
x0 = [thetastart;[0;0;0]];
xT = [thetaend;[0;0.0;0.0]];

% pfl joint trajectory
pfl_n = 3;
Tf = 1.2;
pfl_N= 2000;
method = 3 ;
traj = JointTrajectory(thetastart, thetaend, Tf, pfl_N, method);
thetamat = traj;
dthetamat = zeros(pfl_N, pfl_n);
ddthetamat = zeros(pfl_N, pfl_n);
pfl_dt = Tf / (pfl_N - 1);
for i = 1: pfl_N - 1
  dthetamat(i + 1, :) = (thetamat(i + 1, :) - thetamat(i, :)) / pfl_dt;
  ddthetamat(i + 1, :) = (dthetamat(i + 1, :) - dthetamat(i, :)) / pfl_dt;
end
sim('monkey_pfl');

% load init control from pfl
t = 0:dt:T;
% size(out_u_pfl.Data) 1201 2 , not very good. unify directions of arrays
% to be 2xN
init_u = interp1(out_u_pfl.Time, out_u_pfl.Data,t);
init_u = init_u';

figure(1)
plot(t, init_u(1,:),t, init_u(2,:))
title('PFL Torque');

[pfl_x_list, J] = monkey_complete_run(x0, xT, init_u, dt, step, Q, R, Qf);
dd = sprintf('Cost for PFL strategy %6.5f\n',J);
disp(dd);

out_u_time = t;
out_u_data = [init_u ];
sim('monkey_use_out_u');

% iLQR step
new_u_list = init_u;
new_x_list = pfl_x_list;
% TODO add random noise for initial u
x0 = x0 + 1e-5*randn(size(x0));

total_iter = 100;
lamb = 6.0; % regularization parameter
sim_new_trajectory  = 1;

for i = 1:total_iter
%     try
        [new_x_list,new_u_list, J, lamb, sim_new_trajectory, converge] = ...
            monkey_iLQR_iteration(x0, xT, new_u_list, new_x_list, dt, step, Q, R, Qf, lamb, sim_new_trajectory); 
        dd = sprintf('Iteration %d, cost %6.5f\n', i, J);
        disp(dd);
        
%         out_u_time = t;
%         out_u_data = new_u_list;
%         sim('monkey_use_out_u');
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
sim('monkey_use_out_u');






