syms th1 th2 th3 dth1 dth2 dth3 real
syms tau1 tau2 real

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


thetalist = [th1;th2;th3];
dthetalist = [dth1;dth2;dth3];
action = [0;tau1; tau2];
M = MassMatrix_sym(thetalist, Mlist, Glist, Slist);
M_true = MassMatrix([1;1;1], Mlist, Glist, Slist);
M_true
eval(subs(M,[th1 th2 th3],[1 1 1]))
inv(M_true)
M_inv = inv(M);
eval(subs(M_inv,[th1 th2 th3],[1 1 1]))
grav = GravityForces_sym(thetalist, g, Mlist, Glist, Slist);
grav_true = GravityForces([1;1;1], g, Mlist, Glist, Slist);
grav_true
eval(subs(grav,[th1 th2 th3],[1 1 1]))


c = VelQuadraticForces_sym(thetalist, dthetalist, Mlist, Glist, Slist);
c_true = VelQuadraticForces([1;1;1], [1;1;1], Mlist, Glist, Slist);
c_true
eval(subs(c,[th1 th2 th3 dth1 dth2 dth3],[1 1 1 1 1 1]))


ddthetalist = simplify(M_inv) ...
              * (action - simplify(c) ...
                 - simplify(grav) ...
                 - simplify(EndEffectorForces_sym(thetalist, Ftip, Mlist, Glist, ...
                                     Slist)));
                                 
A = jacobian(ddthetalist,[th1 th2 th3 dth1 dth2 dth3]);   

B = jacobian(ddthetalist,[tau1 tau2]);  

matlabFunction(A, 'Vars' ,[th1 th2 th3 dth1 dth2 dth3 tau1 tau2],'File','A_func','Comments','Version: 1.1');
matlabFunction(B, 'Vars' ,[th1 th2 th3 dth1 dth2 dth3 tau1 tau2],'File','B_func','Comments','Version: 1.1');