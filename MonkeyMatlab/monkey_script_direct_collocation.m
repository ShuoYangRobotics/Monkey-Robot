% this script accompanies monkey_use_out_u, it use direct collocation method to
% calculate a trajectory out_u. and then monkey_use_out_u will use this
% trajectory to control it

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


% total time should be 1.2, then 1.2/200=0.006
step = 100;
dt = 0.012;
T= step*dt;

init_angle_left_hand = -45/180*pi;
init_angle_left_shoulder = 45/180*pi;
init_angle_right_shoulder = 45/180*pi;

thetastart = [init_angle_left_hand; init_angle_left_shoulder; init_angle_right_shoulder];
x = [thetastart;[0;0;0]];

u = zeros (1 , step*2 );

% load init control from pfl
load('out_u_pfl_0225.mat');
t = 0:dt:1.2-dt;
init_u = interp1(out_u_pfl.Time, out_u_pfl.Data,t);
for i=1:step
    u(2*(i-1)+1) = init_u(i,1);
    u(2*(i-1)+2) = init_u(i,2);
end
figure(1)
init_x_list = monkey_state_run(u);plot(init_x_list(1,:),init_x_list(3,:))
title('Initial');
state_u = [reshape(init_x_list,[],1)' u];

A = []; b = []; Aeq = []; beq = [];

options =optimoptions(@fmincon, 'TolFun', 0.00001,...
    'MaxIter', 100, ...
    'MaxFunEvals', 100000, 'Display', 'iter', ...
    'DiffMinChange', 0.001, 'Algorithm', 'sqp');

nonlcon = @monkey_direct_collocation_constraint_func;
lb_state = -Inf*ones(1,step*6);
ub_state = Inf*ones(1,step*6);
lb_u = -6.5*ones(1,step*2);
ub_u = 6.5*ones(1,step*2);
lb = [lb_state lb_u];
ub = [ub_state ub_u];
% out_u = fmincon(@monkey_cost_func,u,A,b,Aeq,beq,lb,ub,nonlcon,options);
out_state_u = fmincon(@monkey_cost_func_direct_collocation,state_u,A,b,Aeq,beq,lb,ub,nonlcon,options);

state = out_state_u(1,1:6*step);
out_u = out_state_u(1,6*step+1:end);
x_list = monkey_state_run(out_u);
% u_1 = out_u(1:2:end); u_2 = out_u(2:2:end);plot(u_1);hold on; plot(u_2)
figure(2)
plot(x_list(1,:),x_list(3,:))
title('Final');

out_u_time = 0:dt:1.2;
out_u_data = [[out_u(1:2:end);out_u(2:2:end)] [0;0]];



