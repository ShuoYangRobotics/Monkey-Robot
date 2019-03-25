script_Monkey_proto_basic;

%% start and end locations
xz_start = [-0.4113;
                  0];
              
xz_end = [0.5;
            0];

%% kinematics
L = (290.81)*10^-3;
w1 = [0;-1;0];
pq1 = [0;0;0];
w2 = [0;-1;0];
pq2 = [0;0;-L];
w3 = [0;1;0];
pq3 = [0;(-68.82-35)*10^-3;-L];
Slist = [[w1;-cross(w1,pq1)] [w2;-cross(w2,pq2)] [w3;-cross(w3,pq3)]];

%% IK
        
r1 = norm(xz_start);
r2 = norm(xz_end);

init_angle_left_hand = -asin(r1/2/L)+atan(xz_start(2)/xz_start(1));
init_angle_left_shoulder = 0/180*pi;
init_angle_right_shoulder = -2*asin(r1/2/L);

end_angle_left_hand = asin(r2/2/L)+atan(xz_end(2)/xz_end(1));
end_angle_left_shoulder = 0/180*pi;
end_angle_right_shoulder = 2*asin(r2/2/L)-2*pi;        

%% trajectoru
thetastart = [init_angle_left_hand; init_angle_left_shoulder; init_angle_right_shoulder];
thetaend = [end_angle_left_hand; end_angle_left_shoulder; end_angle_right_shoulder];
x0 = [thetastart;[0;0;0]];
xT = [thetaend;[0;0.0;0.0]];

T= 0.66;
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
%% FK
% home configuration
home_config = [[  1,  0,   0, 0]; 
               [  0,  1,   0, (-68.82-35)*10^-3]; 
               [  0,  0,   1, 0]; 
               [ 0,  0,  0, 1]];
 
thetalist = [-45/180*pi;
                       0;
             -90/180*pi];
         
% T = FKinSpace(home_config, Slist, thetalist);

x = zeros(size(thetamat,1),1);
z = zeros(size(thetamat,1),1);

for i = 1: size(thetamat,1)
    T = FKinSpace(home_config, Slist, thetamat(i,:));
    x(i) = T(1,4);
    z(i) = T(3,4);
end
plot(x,z);

           
