% Derive_acrobot.m
%
% This script derives the equations of motion for the simple acrobot robot:
% a double pendulum with a motor at the elbow joint. 
%

syms q1 q2 dq1 dq2 ddq1 ddq2 'real'   % states
syms u 'real' % actuation
syms m1 m2 g l1 l2 'real' % physical parameters

%%%% Unit vectors:
i = sym([1;0]);
j = sym([0;1]);

e1 = cos(q1)*(-j) + sin(q1)*(i);    % shoulder -> elbow
e2 = cos(q2)*(-j) + sin(q2)*(i);    % elbow -> wrist

%%%% State vectors:
z = [q1;q2;dq1;dq2];
dz = [dq1;dq2;ddq1;ddq2];
dq = [dq1;dq2];

%%%% Kinematics:
p1 = l1*e1;
p2 = p1 + l2*e2;

dp1 = jacobian(p1,z)*dz;  %Chain rule to get velocity of elbow joint
dp2 = jacobian(p2,z)*dz; 

ddp1 = jacobian(dp1,z)*dz;  
ddp2 = jacobian(dp2,z)*dz; 



%%%% Write out dynamics in matrix form:   MM*ddq = ff
ddq = [ddq1;ddq2];

% From mathematica acrobot
MM = [l1.^2.*m1+l1.^2.*m2 l1*l2*m2*cos(q1-q2);
      l1*l2*m2*cos(q1-q2) m2*l2^2];
ff = [2*m1*g*l1*sin(q1)+m2*l1*l2*sin(q1-q2)*dq2^2;
        m1*l2*g*sin(q2)-m2*l1*l2*sin(q1-q2)*dq1^2+u];

%%%% Generate an optimized matlab function for dynamics:
matlabFunction(MM(1,1),MM(1,2),MM(2,1),MM(2,2),ff(1),ff(2),...
    'file','autoGen_acrobotDynamics.m',...
    'vars',{q1,q2,dq1,dq2,u,m1,m2,g,l1,l2},...
    'outputs',{'M11','M12','M21','M22','f1','f2'});

%%%% Compute the energy of the system:
U = m1*g*dot(p1,j) + m2*g*dot(p2,j);   %Potential Energy
T = 0.5*m1*dot(dp1,dp1) + 0.5*m2*dot(dp2,dp2);   %Kinetic Energy

%%%% Generate an optimized matlab function for energy:
matlabFunction(U,T,...
    'file','autoGen_acrobotEnergy.m',...
    'vars',{q1,q2,dq1,dq2,m1,m2,g,l1,l2},...
    'outputs',{'U','T'});

%%%% Generate a function for computing the kinematics:
matlabFunction(p1,p2,dp1,dp2,...
    'file','autoGen_acrobotKinematics.m',...
    'vars',{q1,q2,dq1,dq2,l1,l2},...
    'outputs',{'p1','p2','dp1','dp2'});