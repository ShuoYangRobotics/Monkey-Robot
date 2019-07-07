addpath('ModernRobotics\packages\Matlab\mr');
% Monkey_all_DataFile;
%% calculate dynamic model
link_N=3;
g = [0; 0; -9.8];
Ftip = zeros(6, 1); 
arm_mass =  0.38466;
body_mass = 2.11051;
arm_Ixx =   1694588*10^-9;
arm_Iyy =   1752068*10^-9;  
arm_Izz =    235533*10^-9;

body_Ixx = 17128654*10^-9;
body_Iyy =  6112223*10^-9;
body_Izz = 12304473*10^-9;
% robot structure
% be careful about the position of these frames 
M1 = [eye(3) [ 0*10^-3; 9.374*10^-3; -133.768*10^-3]; [0 0 0 1]];
M2 = [eye(3) [ 0*10^-3; (19.72-73.26)*10^-3; (-309.81-40.91)*10^-3]; [0 0 0 1]];
M3 = [eye(3) [ 0*10^-3; (-133.7+11.84)*10^-3; (-309.81+176.07)*10^-3]; [0 0 0 1]];
M4 = [eye(3) [ 0; (-102.7)*10^-3; 0]; [0 0 0 1]];

M01 = M1;
M12 = inv(M1)*M2;
M23 = inv(M2)*M3;
M34 = inv(M3)*M4;

I1 = [arm_Ixx              0              0;
            0        arm_Iyy              0;
            0              0        arm_Izz ];
        
G1 = [I1 zeros(3,3);
      zeros(3,3) diag([arm_mass, arm_mass, arm_mass])];
  
I2 = [   body_Ixx              0               0;
                0       body_Iyy               0;
                0              0        body_Izz ];
G2 = [I2 zeros(3,3);
      zeros(3,3) diag([body_mass, body_mass, body_mass])];
  
I3 = [arm_Ixx                 0                  0;
            0           arm_Iyy                  0;
            0                 0            arm_Izz ];
G3 = [I3 zeros(3,3);
      zeros(3,3) diag([arm_mass, arm_mass, arm_mass])];

Glist = cat(3, G1, G2, G3);
Mlist = cat(3, M01, M12, M23, M34); 


w1 = [0;-1;0];
pq1 = [0;0;0];
% before 4-19, this is 0;-1;0.. but on 4-19 i found this should be 0;1;0
w2 = [0;1;0];
pq2 = [0;0;(-309.8)*10^-3];
w3 = [0;1;0];
pq3 = [0;0;(-309.8)*10^-3];
Slist = [[w1;-cross(w1,pq1)] [w2;-cross(w2,pq2)] [w3;-cross(w3,pq3)]];