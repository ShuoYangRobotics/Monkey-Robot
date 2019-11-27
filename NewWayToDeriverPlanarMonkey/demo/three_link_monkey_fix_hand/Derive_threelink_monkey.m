% Derive_three_link_monkey.m
%
%

syms q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3 'real'   % states
syms u1 u2 'real' % actuation
syms m1 m2 m3 g l1 l2 l3 'real' % physical parameters
empty = sym('empty','real');   %Used for vectorization, user should pass a vector of zeros
t = sym('t','real');  %dummy continuous time


%%%% State vectors:
z = [q1;q2;q3;dq1;dq2;dq3];
dz = [dq1;dq2;dq3;ddq1;ddq2;ddq3];
dq = [dq1;dq2;dq3];
ddq = [ddq1;ddq2;ddq3];


nz = [t;z;u1;u2];   % time-varying vector of inputs

%%%% Kinematics:
p1 = [0.5*l1*sin(q1);-0.5*l1*cos(q1)];
p1e = [l1*sin(q1);-l1*cos(q1)];
p2 = [l1*sin(q1)  + 0.5*l2*sin(q1+q2);-l1*cos(q1)-0.5*l2*cos(q1+q2)];
p2e = [l1*sin(q1)  + l2*sin(q1+q2);-l1*cos(q1) - l2*cos(q1+q2)];
p3 = [l1*sin(q1)  - 0.5*l3*sin(q1+q2+q3);-l1*cos(q1)+0.5*l3*cos(q1+q2+q3)];
p3e = [l1*sin(q1)  - l3*sin(q1+q2+q3);-l1*cos(q1) + l3*cos(q1+q2+q3)];

v1 = [0.5*l1*cos(q1)*dq1; 0.5*l1*sin(q1)*dq1];
v2 = [l1*cos(q1)*dq1+0.5*l2*cos(q1+q2)*(dq1+dq2); l1*sin(q1)*dq1+0.5*l2*sin(q1+q2)*(dq1+dq2)];
v3 = [l1*cos(q1)*dq1-0.5*l2*cos(q1+q2+q3)*(dq1+dq2+dq3);
      l1*sin(q1)*dq1-0.5*l2*sin(q1+q2+q3)*(dq1+dq2+dq3)];
  
I1 = 1/12*m1*l1^2;
I2 = 1/12*m2*l2^2;
I3 = 1/12*m3*l3^2;

% From mathematica "fixhand monkey three link"
% currently there is no good way to convert mathematica to matlab, I did
% this manually 
MM = [I1+0.25.*l1.^2.*m1+l1.^2.*m2+0.25.*l2.^2.*m2+l1.^2.*m3+0.25.*l3.^2.*m3+l1.*l2.*m2.*cos(q2)+(-1).*l1.*l3.*m3.*cos(q2+q3) ...
      0.25.*l2.^2.*m2+0.25.*l3.^2.*m3+0.5.*l1.*l2.*m2.*cos(q2)+(-0.5).*l1.*l3.*m3.*cos(q2+q3) ...
      0.25.*l3.^2.*m3+(-0.5).*l1.*l3.*m3.*cos(q2+q3);
      0.25.*l2.^2.*m2+0.25.*l3.^2.*m3+0.5.*l1.*l2.*m2.*cos(q2)+(-0.5).*l1.*l3.*m3.*cos(q2+q3) ...
      I2+0.25.*l2.^2.*m2+0.25.*l3.^2.*m3 ...
      0.25.*l3.^2.*m3;
      0.25.*l3.^2.*m3+(-0.5).*l1.*l3.*m3.*cos(q2+q3) ...
      0.25.*l3.^2.*m3 ...
      I3+0.25.*l3.^2.*m3];   
  
ff = [(-1.5).*g.*l1.*m1.*sin(q1)+(-1).*g.*l1.*m3.*sin(q1)+(-0.5).*g.*l2.*m1.*sin(q1+q2)+0.5.*g.*l3.*m3.*sin(q1+q2+q3)+(-1).*dq3.*(0.5.*l1.*l3.*m3.*sin(q2+q3).*dq1+0.5.*l1.*l3.*m3.*sin (q2+q3).*dq2+0.5.*l1.*l3.*m3.*sin (q2+q3).*dq3)+(-1).*dq2.*((1/2).*((-1).*l1.*l2.*m2.*sin(q2)+l1.*l3.*m3.*sin(q2+q3)).*dq1+((-0.5).*l1.*l2.*m2.*sin(q2)+0.5.*l1.*l3.*m3.*sin(q2+q3)).*dq2+0.5.*l1.*l3.*m3.*sin (q2+q3).*dq3)+(-1).*dq1.*((1/2).*((-1).*l1.*l2.*m2.*sin(q2)+l1.*l3.*m3.*sin(q2+q3)).*dq2+0.5.*l1.*l3.*m3.*sin(q2+q3).*dq3);
      u1+(-0.5).*g.*l2.*m1.*sin(q1+q2)+0.5.*g.*l3.*m3.*sin(q1+q2+q3)+(-1).*(0.5.*l1.*l2.*m2.*sin(q2)+(-0.5).*l1.*l3.*m3.*sin(q2+q3)).*dq1.^2;
      u2+0.5.*g.*l3.*m3.*sin(q1+q2+q3)+0.5.*l1.*l3.*m3.*sin(q2+q3).*dq1.^2];

%%%% Generate an optimized matlab function for dynamics:
matlabFunction(MM(1,1),MM(1,2),MM(1,3),MM(2,1),MM(2,2),MM(2,3),MM(3,1),MM(3,2),MM(3,3),ff(1),ff(2),ff(3),...
    'file','autoGen_monkeyDynamics.m',...
    'vars',{q1,q2,q3,dq1,dq2,dq3,u1,u2,m1,m2,m3,g,l1,l2,l3},...
    'outputs',{'M11','M12','M13','M21','M22','M23','M31','M32','M33','f1','f2','f3'});

%%%% Compute gradients these code are from kelly's fivelink biped
% I cannot understand this yet
[m, mi, mz, mzi, mzd] = computeGradients(MM,nz,empty);
[f, fi, fz, fzi, fzd] = computeGradients(ff,nz,empty);

% Write function file:
matlabFunction(m, mi, f, fi,...   %dynamics
    mz, mzi, mzd, fz, fzi, fzd,...  %gradients
    'file','autoGen_dynSs.m',...
    'vars',{...
    'q1','q2','q3'...
    'dq1','dq2','dq3',...
    'u1','u2',...
    'm1','m2','m3',...
    'l1','l2','l3',...
    'g','empty'});

%%%% Compute the energy of the system:
U = m1*g*p1(2) + m2*g*p2(2) + m3*g*p3(2);   %Potential Energy
T = 0.5*m1*(v1'*v1) + 0.5*m2*(v2'*v2) + 0.5*m3*(v3'*v3)+...
    0.5*I1*dq1^2+0.5*I2*dq2^2+0.5*I3*dq3^2;   %Kinetic Energy

%%%% Generate an optimized matlab function for energy:
matlabFunction(U,T,...
    'file','autoGen_monkeyEnergy.m',...
    'vars',{q1,q2,q3,dq1,dq2,dq3,m1,m2,m3,g,l1,l2,l3},...
    'outputs',{'U','T'});

%%%% Generate a function for computing the kinematics:
matlabFunction(p1,p1e,p2,p2e,p3,p3e,v1,v2,v3,...
    'file','autoGen_monkeyKinematics.m',...
    'vars',{q1,q2,q3,dq1,dq2,dq3,m1,m2,m3,g,l1,l2,l3},...
    'outputs',{'p1','p1e','p2','p2e','p3','p3e','v1','v2','v3'});