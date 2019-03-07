% this script accompanies moneky_pfl, it use partial feedback linearization
% and a naive trajectory generation method to control the robot

monkey_script_basic;

% init angles
init_angle_left_hand = -45/180*pi;
init_angle_left_shoulder = 45/180*pi;
init_angle_right_shoulder = 45/180*pi;

end_angle_left_hand = 45/180*pi;
end_angle_left_shoulder = -45/180*pi;
end_angle_right_shoulder = (45+270)/180*pi;


% calculate 
N=3;
g = [0; 0; -9.8];
Ftipmat = zeros(N, 6); 

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

thetalist = [-pi/4; 0; 0];
dthetalist = [0; 0; 0];
M = MassMatrix(thetalist, Mlist, Glist, Slist)
grav = GravityForces(thetalist, g, Mlist, Glist, Slist)
c = VelQuadraticForces(thetalist, dthetalist, Mlist, Glist, Slist)

% joint trajectory
n = 3;
thetastart = [init_angle_left_hand; init_angle_left_shoulder; init_angle_right_shoulder];
thetaend = [end_angle_left_hand; end_angle_left_shoulder; end_angle_right_shoulder];
Tf = 1.0;
N= 2000;
method = 3 ;
traj = JointTrajectory(thetastart, thetaend, Tf, N, method);
thetamat = traj;
dthetamat = zeros(N, n);
ddthetamat = zeros(N, n);
dt = Tf / (N - 1);
for i = 1: N - 1
  dthetamat(i + 1, :) = (thetamat(i + 1, :) - thetamat(i, :)) / dt;
  ddthetamat(i + 1, :) = (dthetamat(i + 1, :) - dthetamat(i, :)) / dt;
end
%monkey_joint_floating_base_ctrl_gen;
