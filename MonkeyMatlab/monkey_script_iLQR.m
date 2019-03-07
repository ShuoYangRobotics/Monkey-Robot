% this script accompanies monkey_use_out_u, it use iLQR method to
% calculate a trajectory out_u. and then monkey_use_out_u will use this
% trajectory to control it
% 2019-03-06 ideally, this method should generate a trajectory that is
% identical to the direct collocation method
% 2019-03-07 it does not give very good control strategy...
% the reason may be 

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
step = 300;
T= 1.2;
dt = T/step;

init_angle_left_hand = -45/180*pi;
init_angle_left_shoulder = 45/180*pi;
init_angle_right_shoulder = 45/180*pi;

end_angle_left_hand = 45/180*pi;
end_angle_left_shoulder = -45/180*pi;
end_angle_right_shoulder = (45+270)/180*pi;

thetastart = [init_angle_left_hand; init_angle_left_shoulder; init_angle_right_shoulder];
thetaend = [end_angle_left_hand; end_angle_left_shoulder; end_angle_right_shoulder];
x = [thetastart;[0;0;0]];
xT = [thetaend;[0;0;0]];

u = zeros (1 , step*2 );

% load init control from pfl
load('out_u_pfl_0225.mat');
t = 0:dt:1.2-dt;
init_u = interp1(out_u_pfl.Time, out_u_pfl.Data,t);
for i=1:step
    u(2*(i-1)+1) = init_u(i,1);
    u(2*(i-1)+2) = init_u(i,2);
end
t = [t T];
figure(1)
init_x_list = monkey_state_run(u,dt);
%plot(init_x_list(1,:),init_x_list(3,:))
plot(t, [init_u(:,1)' 0],t, [init_u(:,2)' 0])
title('PFL Torque');

% load direct collocation control to compare
load('0226_direct_collocation');
t = 0:dt:T-dt;
final_u = [interp1(out_u_time, out_u_data(1,:),t);interp1(out_u_time, out_u_data(2,:),t);];
for i=1:step
    u(2*(i-1)+1) = final_u(1,i);
    u(2*(i-1)+2) = final_u(2,i);
end

t = [t T];
figure(2)
final_x_list = monkey_state_run(u,dt);
%plot(final_x_list(1,:),final_x_list(3,:))
plot(t, [final_u(1,:) 0],t, [final_u(2,:) 0])
title('Direct Collocation Torque');

% forward pass with new_u
% calculate cost for reference
Q = 0*eye(6);
R = 1*eye(2);
Qf = 8270*eye(6);
J = 0;
new_x_list = zeros(6,step+1);
new_x_list(:,1) = final_x_list(:,1);
for i = 1:step
    tmp_state = new_x_list(:,i);
    x_dot = monkey_dyn_func(tmp_state, [0;final_u(:,i)]);
    new_x_list(:,i+1) = new_x_list(:,i) + dt*x_dot;

    J = J + new_x_list(:,i)'*Q*new_x_list(:,i) + final_u(:,i)'*R*final_u(:,i);
end

J = J + (new_x_list(:,end) - xT)'*Qf*(new_x_list(:,end) - xT);
J

% iLQR step
% new_u_list = final_u;
% new_x_list = final_x_list;
new_u_list = init_u';
new_x_list = init_x_list;
total_iter = 650;

for i = 1:total_iter
    [new_x_list,new_u_list] = monkey_iLQR_iteration(new_x_list, xT, new_u_list, step, dt);    
    
%     out_u_time = t;
%     out_u_data = [new_u_list [0;0]];
%     sim('monkey_use_out_u');

end

plot(t, [new_u_list(1,:) 0],t, [new_u_list(2,:) 0])
title('iLQR Torque');








