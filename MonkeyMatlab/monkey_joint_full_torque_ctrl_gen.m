n = 3;
thetastart = [0; 0; 0];
thetaend = [pi/6; -pi/6; -pi/4];
Tf = 2;
N= 2000;
method = 5 ;
traj = JointTrajectory(thetastart, thetaend, Tf, N, method);
thetamat = traj;
dthetamat = zeros(N, n);
ddthetamat = zeros(N, n);
dt = Tf / (N - 1);
for i = 1: N - 1
  dthetamat(i + 1, :) = (thetamat(i + 1, :) - thetamat(i, :)) / dt;
  ddthetamat(i + 1, :) = (dthetamat(i + 1, :) - dthetamat(i, :)) / dt;
end
%Initialise robot descripstion 
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


taumat = InverseDynamicsTrajectory(thetamat, dthetamat, ddthetamat, ...
                                 g, Ftipmat, Mlist, Glist, Slist);
%Output using matplotlib to plot the joint forces/torques
time=0: dt: Tf;
plot(time, taumat(:, 1), 'b')
hold on
plot(time, taumat(:, 2), 'g')
plot(time, taumat(:, 3), 'r')
title('Plot for Torque Trajectories')
xlabel('Time')
ylabel('Torque')
legend('Tau1', 'Tau2', 'Tau3')