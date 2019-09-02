function monkey_ilqg
% init dynamics
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


%% %%%%%%%%%%%%%%%%%%%%% set up control goal
% start and end locations

dist1 = -0.4;
release_height = -0.00;

dist2 = 0.4;
reach_height = 0.00;

xz_start = [dist1;
            release_height];
              
xz_end = [dist2;
          reach_height];
      
% save_file_name = strcat('mod_test',num2str(abs(dist1)),'_',num2str(abs(release_height)),'_',num2str(abs(dist2)),'_',num2str(abs(reach_height)),'.mat');
% save_file_name(save_file_name=='.')=[];

% kinematics
L = (309.8)*10^-3;

% IK
        
r1 = norm(xz_start);
r2 = norm(xz_end);

offset_body_angle = 0/180*pi;


init_angle_left_hand = -asin(r1/2/L)+atan(xz_start(2)/xz_start(1));
init_angle_left_shoulder = 0/180*pi - offset_body_angle;
init_angle_right_shoulder = -2*asin(r1/2/L) + offset_body_angle;

end_angle_left_hand = asin(r2/2/L)+atan(xz_end(2)/xz_end(1));
end_angle_left_shoulder = 0/180*pi;
end_angle_right_shoulder = 2*asin(r2/2/L)-2*pi;        
%%%%%%%%%%%%%%%%%%%%%%%

%% trajectory parameters
step = 300;
T= 0.66;
dt = T/step;


Q = 0*eye(6);
Q(4,4) = 0.02;
Q(5,5) = 0.02;
Q(6,6) = 0.02;

R = 0.3*eye(2);
Qf = 6400*eye(6);
Qf(4,4) = 0;
Qf(5,5) = 0;
Qf(6,6) = 0;

thetastart = [init_angle_left_hand; init_angle_left_shoulder; init_angle_right_shoulder];
thetaend = [end_angle_left_hand; end_angle_left_shoulder; end_angle_right_shoulder];
x0 = [thetastart;[0;0;0]];
xg = [thetaend;[0;0.0;0.0]];

%% load init control from file
tgt_state = [];
tgt_u = [];
tgt_time = []; 
load('mod_test04_0_04_0mat.mat');
% this file contains tgt_t tgt_u tgt_state

t = 0:dt:T;
% inteprete trajectory
init_u = interp1(tgt_time', tgt_u',t);
init_u = init_u';

init_x_list = interp1(tgt_time', tgt_state',t);
init_x_list = init_x_list';
% init_x_list(:,1) = x0;
% init_u(:,end) = [nan;nan];


% [xn, A, B] = update(init_x_list(:,1),init_u(:,1));
% 
% [A, B] = grad(init_x_list(:,1),init_u(:,1));
% [fxx, fxu, fuu] = hessian(init_x_list(:,1),init_u(:,1));
% 
% [f,c,fx,fu,fxx,fxu,fuu,cx,cu,cxx,cxu,cuu] = monkey_dyn_cst(init_x_list,init_u);
% run the optimization
Op.parallel = false;
Op.tolFun = 1e-10;
Op.tolGrad = 1e-10;
Op.maxIter = 150;
xhistd = 0;
uhistd = 0;
costd = 0;
traced = struct();

% [xhisti,uhisti,~,~,~,costi,tracei] = iLQG(@(x,u,i) monkey_dyn_cst(x,u), x0, init_u, Op);
[xhistu,uhistu,~,~,~,costu,traceu] = UDP(@(x,u,i) monkey_dyn_cst_udp(x,u), x0, init_u, Op, @Monkey_all_dyn_func, dt, .01);


% dynamics
function [xn, A, B] = update(x,u)
    u = [0;u];
    %4th order Runge-Kutta Step
    xdot1 = Monkey_all_dyn_func(x,u, g, Ftip, Mlist, Glist, Slist);
    xdot2 = Monkey_all_dyn_func(x+.5*dt*xdot1,u, g, Ftip, Mlist, Glist, Slist);
    xdot3 = Monkey_all_dyn_func(x+.5*dt*xdot2,u, g, Ftip, Mlist, Glist, Slist);
    xdot4 = Monkey_all_dyn_func(x+dt*xdot3,u, g, Ftip, Mlist, Glist, Slist);
    
    xn = x + (dt/6)*(xdot1 + 2*xdot2 + 2*xdot3 + xdot4);
    
    if nargout > 1
        n = length(x);
        m = length(u);
        
        delta = 1e-7;
        Dx = delta*eye(n);
        Du = delta*eye(m);
        
        A1 = zeros(n,n);
        A2 = A1;
        A3 = A1;
        A4 = A1;
        B1 = zeros(n,m);
        B2 = B1;
        B3 = B1;
        B4 = B1;
        for j = 1:n
            xp1 = Monkey_all_dyn_func(x+Dx(:,j),u, g, Ftip, Mlist, Glist, Slist);
            xm1 = Monkey_all_dyn_func(x-Dx(:,j),u, g, Ftip, Mlist, Glist, Slist);
            A1(:,j) = (xp1-xm1)/(2*delta);
            
            xp2 = Monkey_all_dyn_func(x+.5*dt*xdot1+Dx(:,j),u, g, Ftip, Mlist, Glist, Slist);
            xm2 = Monkey_all_dyn_func(x+.5*dt*xdot1-Dx(:,j),u, g, Ftip, Mlist, Glist, Slist);
            A2(:,j) = (xp2-xm2)/(2*delta);
            
            xp3 = Monkey_all_dyn_func(x+.5*dt*xdot2+Dx(:,j),u, g, Ftip, Mlist, Glist, Slist);
            xm3 = Monkey_all_dyn_func(x+.5*dt*xdot2-Dx(:,j),u, g, Ftip, Mlist, Glist, Slist);
            A3(:,j) = (xp3-xm3)/(2*delta);
            
            xp4 = Monkey_all_dyn_func(x+dt*xdot3+Dx(:,j),u, g, Ftip, Mlist, Glist, Slist);
            xm4 = Monkey_all_dyn_func(x+dt*xdot3-Dx(:,j),u, g, Ftip, Mlist, Glist, Slist);
            A4(:,j) = (xp4-xm4)/(2*delta);
        end
        for j = 1:m
            xp1 = Monkey_all_dyn_func(x,u+Du(:,j), g, Ftip, Mlist, Glist, Slist);
            xm1 = Monkey_all_dyn_func(x,u-Du(:,j), g, Ftip, Mlist, Glist, Slist);
            B1(:,j) = (xp1-xm1)/(2*delta);
            
            xp2 = Monkey_all_dyn_func(x+.5*dt*xdot1,u+Du(:,j), g, Ftip, Mlist, Glist, Slist);
            xm2 = Monkey_all_dyn_func(x+.5*dt*xdot1,u-Du(:,j), g, Ftip, Mlist, Glist, Slist);
            B2(:,j) = (xp2-xm2)/(2*delta);
            
            xp3 = Monkey_all_dyn_func(x+.5*dt*xdot2,u+Du(:,j), g, Ftip, Mlist, Glist, Slist);
            xm3 = Monkey_all_dyn_func(x+.5*dt*xdot2,u-Du(:,j), g, Ftip, Mlist, Glist, Slist);
            B3(:,j) = (xp3-xm3)/(2*delta);
            
            xp4 = Monkey_all_dyn_func(x+dt*xdot3,u+Du(:,j), g, Ftip, Mlist, Glist, Slist);
            xm4 = Monkey_all_dyn_func(x+dt*xdot3,u-Du(:,j), g, Ftip, Mlist, Glist, Slist);
            B4(:,j) = (xp4-xm4)/(2*delta);
        end
        
        A = (eye(n)+(dt/6)*A4)*(eye(n)+(dt/3)*A3)*(eye(n)+(dt/3)*A2)*(eye(n)+(dt/6)*A1);
        B = (dt/6)*B4 + (eye(n)+(dt/6)*A4)*(dt/3)*B3 + (eye(n)+(dt/6)*A4)*(eye(n)+(dt/3)*A3)*(dt/3)*B2 + (eye(n)+(dt/6)*A4)*(eye(n)+(dt/3)*A3)*(eye(n)+(dt/3)*A2)*(dt/6)*B1;
        
    end
end

function [A, B] = grad(x,u)
    n = length(x);
    m = length(u);

    A = zeros(n,n);
    B = zeros(n,m);
    
    delta = 1e-7;
    Dx = delta*eye(n);
    Du = delta*eye(m);
    
    for j = 1:n
        xp = update(x+Dx(:,j), u);
        xm = update(x-Dx(:,j), u);
        A(:,j) = (xp-xm)/(2*delta);
    end
    for j = 1:m
        xp = update(x, u+Du(:,j));
        xm = update(x, u-Du(:,j));
        B(:,j) = (xp-xm)/(2*delta);
    end
end

function [fxx, fxu, fuu] = hessian(x,u)
    n = length(x);
    m = length(u);
    
    delta = 1e-5;
    Dx = delta*eye(n);
    Du = delta*eye(m);
    
    fxx = zeros(n,n,n);
    fxu = zeros(n,n,m);
    fuu = zeros(n,m,m);
    
    for j = 1:n
        Ap = grad(x+Dx(:,j), u);
        Am = grad(x-Dx(:,j), u);
        fxx(:,:,j) = (Ap-Am)/(2*delta);
    end
    
    for j = 1:m
        [Ap, Bp] = grad(x, u+Du(:,j));
        [Am, Bm] = grad(x, u-Du(:,j));
        fxu(:,:,j) = (Ap-Am)/(2*delta);
        fuu(:,:,j) = (Bp-Bm)/(2*delta);
    end
end

function [f,c,fx,fu,fxx,fxu,fuu,cx,cu,cxx,cxu,cuu] = monkey_dyn_cst(x,u)

% for a positive-definite quadratic, no control cost (indicated by the 
% iLQG function using nans), is equivalent to u=0
N = size(u,2);
n = size(x,1);
m = size(u,1);

f = zeros(n,N);
c = 0;

if nargout == 2
    for k = 1:N
        if(isnan(u(:,k)))
            c = c + (x(:,k)-xg)'*Qf*(x(:,k)-xg);
        else
            f(:,k) = update(x(:,k),u(:,k));
            c = c + (x(:,k)-xg)'*Q*(x(:,k)-xg) + u(:,k)'*R*u(:,k);
        end
    end
else
    A = zeros(n,n,N);
    B = zeros(n,m,N);
    fxx = zeros(n,n,n,N);
    fxu = zeros(n,n,m,N);
    fuu = zeros(n,m,m,N);

    for k = 1:N
       
        if(isnan(u(:,k)))
           
            c = c + (x(:,k)-xg)'*Qf*(x(:,k)-xg);
          
        else
            f(:,k) = update(x(:,k),u(:,k));
            [A(:,:,k), B(:,:,k)] = grad(x(:,k),u(:,k));
            [fxx(:,:,:,k), fxu(:,:,:,k), fuu(:,:,:,k)] = hessian(x(:,k),u(:,k));
            c = c + (x(:,k)-xg)'*Q*(x(:,k)-xg) + u(:,k)'*R*u(:,k);
        end
    end

    fx  = A(:,:,1:end-1);
    fu  = B(:,:,1:end-1);
    cx  = Q*[x(1,:)-xg(1); x(2,:)-xg(2); x(3,:)-xg(3); x(4,:)-xg(4); x(5,:)-xg(5); x(6,:)-xg(6)];
    cx(:,end) = Qf*(x(:,end)-xg);
    cu  = R*u;
    cxx = repmat(Q, [1 1 N]);
    cxx(:,:,N) = Qf;
    cxu = repmat(zeros(n,m), [1 1 N]);
    cuu = repmat(R, [1 1 N]);
end
end

function [f,c,cx,cu,cxx,cxu,cuu] = monkey_dyn_cst_udp(x,u)

% for a positive-definite quadratic, no control cost (indicated by the 
% iLQG function using nans), is equivalent to u=0
N = size(x,2);
n = size(x,1);
m = size(u,1);

f = zeros(n,N);
c = 0;

if nargout == 2
    for k = 1:N
        if(isnan(u(:,k)))
            c = c + (x(:,k)-xg)'*Qf*(x(:,k)-xg);
        else
            f(:,k) = update(x(:,k),u(:,k));
            c = c + (x(:,k)-xg)'*Q*(x(:,k)-xg) + u(:,k)'*R*u(:,k);
        end
    end
else
    cx  = Q*[x(1,:)-xg(1); x(2,:)-xg(2); x(3,:)-xg(3); x(4,:)-xg(4); x(5,:)-xg(5); x(6,:)-xg(6)];
    cx(:,end) = Qf*(x(:,end)-xg);
    cu  = R*u;
    cxx = repmat(Q, [1 1 N]);
    cxx(:,:,N) = Qf;
    cxu = repmat(zeros(n,m), [1 1 N]);
    cuu = repmat(R, [1 1 N]);
    [f,c] = deal([]); 
end
end

end