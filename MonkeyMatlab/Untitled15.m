monkey_script_basic;

m = arm_mass*2 + body_mass;
T= 1.2;
distance = 2*sqrt(2);


step = 300;
dt = T/step;
%% brachiation
load('0226_direct_collocation');
t = 0:dt:T-dt;
final_u = [interp1(out_u_time, out_u_data(1,:),t);interp1(out_u_time, out_u_data(2,:),t);];
for i=1:step
    u(2*(i-1)+1) = final_u(1,i);
    u(2*(i-1)+2) = final_u(2,i);
end
final_x_list = monkey_state_run(u,dt);
% use u and dtheta to calculate V I and power


%% wheel
x0 = [0;0];
xT = [distance;0];
u = 0.0001*ones(1,step);
radius = 0.25;  % radius of wheel
A = []; b = []; Aeq = []; beq = [];
options =optimoptions(@fmincon, 'TolFun', 0.00001,...
    'MaxIter', 1000, ...
    'MaxFunEvals', 1000000, 'Display', 'iter', ...
    'DiffMinChange', 0.001, 'Algorithm', 'interior-point');
nonlcon = []
lb = -17*ones(1,step);
ub = 17*ones(1,step);
out_u = fmincon(@wheel_cost_func,u,A,b,Aeq,beq,lb,ub,[],options);

wheel_x_list = zeros(2,step);
x = [0;0];
wheel_x_list(:,1) = x;

for i = 1:step 
    wheel_x_list(:,i) = x;
    x = x + dt * ([0 1;0 0]*x + [0;1/m/radius]*out_u(i));
end
%% power calculation
% use maxson motor 397172 and gear  203115

kT = 36.9 * 10^-3;
Ra = 0.608;
La = 0.463 * 10^-3;
Jm = 181*10^-7;
taum = 8.07*10^-3;
ke = Jm*Ra/kT/taum;
N = 1;

% power for wheel robot
power = 0;
for i = 2:step
    I = out_u(i)/kT/N;
    dI = (out_u(i)-out_u(i-1))/dt/kT;
    V = I*Ra + dI*La+ke*wheel_x_list(2,i)*N/radius;
    power = power + V*I*dt;
end
power_wheel = power

% power for brachiation robot
power = 0;
for i = 2:step
    %left
    I = final_u(1,i)/kT/N;
    dI = (final_u(1,i)-final_u(1,i-1))/dt/kT;
    V = I*Ra + dI*La+ke*final_x_list(4,i)*N;
    power = power + V*I*dt;
    
    %right
    I = final_u(2,i)/kT/N;
    dI = (final_u(2,i)-final_u(2,i-1))/dt/kT;
    V = I*Ra + dI*La+ke*final_x_list(5,i)*N;
    power = power + V*I*dt;
end
power_robot = power