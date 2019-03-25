function qdot = monkey_dyn_func ( state , action ) 
    % state is q1 q2 q3 q1_dot q2_dot q3_dot
arm_length = 1;
arm_width = 0.1;
arm_depth = 0.1;
arm_density = 100;

body_width = 0.3;
body_depth = 0.15;
body_length = 0.5;
body_density = 90;

% basic calculation
arm_mass = arm_density * arm_length * arm_depth * arm_width;
body_mass = body_density * body_length * body_depth * body_width;

arm_Ixx = 1/12*arm_mass*(arm_width*arm_width + arm_length*arm_length); 
arm_Iyy = 1/12*arm_mass*(arm_depth*arm_depth + arm_length*arm_length); 
arm_Izz = 1/12*arm_mass*(arm_width*arm_width + arm_depth*arm_depth); 


body_Ixx = 1/12*body_mass*(body_width*body_width + body_length*body_length); 
body_Iyy = 1/12*body_mass*(body_depth*body_depth + body_length*body_length); 
body_Izz = 1/12*body_mass*(body_width*body_width + body_depth*body_depth);
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
    thetalist = state(1:3);
    dthetalist = state(4:6);

    ddthetalist = ForwardDynamics(thetalist, dthetalist, action, ...
                                       g, Ftip, Mlist, Glist, Slist);
                                   
% qdot =  q1_dot q2_dot q3_dot q1_ddot q2_ddot dq3_dot                                 
    qdot = [ state(4);
             state(5);
             state(6);
             ddthetalist];
end