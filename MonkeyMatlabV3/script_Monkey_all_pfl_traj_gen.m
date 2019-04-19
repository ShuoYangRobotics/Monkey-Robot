% init dynamics
script_Monkey_all_basic;


%%%%%%%%%%%%%%%%%%%%%%% set up control goal
init_angle_left_hand = -45/180*pi;
init_angle_left_shoulder = 0/180*pi;
init_angle_right_shoulder = -90/180*pi;

end_angle_left_hand = 45/180*pi;
end_angle_left_shoulder = 0/180*pi;
end_angle_right_shoulder = -(90+90+90)/180*pi;
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

% pfl joint trajectory
pfl_n = 3;
Tf = T;
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

% sim('Monkey_all_pfl','StopTime', num2str(T));

%% load init control from pfl
t = 0:dt:T;
% size(out_u_pfl.Data) 1201 2 , not very good. unify directions of arrays
% to be 2xN
init_u = interp1(out_u_pfl.Time, out_u_pfl.Data,t);
init_u = init_u';
Q = 0*eye(6);
R = 0.4*eye(2);
Qf = 2000*eye(6);
Qf(4,4) = 0;
% Qf(5,5) = 0;
% Qf(6,6) = 0;
