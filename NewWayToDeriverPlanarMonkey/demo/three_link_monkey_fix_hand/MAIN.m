%MAIN.m  --  solve swing-up problem for acrobot
%
% This script finds the minimum torque-squared trajectory to swing up the
% acrobot robot: a double pendulum with a motor between the links
%
%

clc; clear;
addpath ../../

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                  Parameters for the dynamics function                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
dyn.m1 = 1;  % elbow mass
dyn.m2 = 1; % wrist mass
dyn.m3 = 1; % wrist mass
dyn.g = 9.81;  % gravity
dyn.l1 = 1;   % length of first link
dyn.l2 = 0.5;   % length of second link
dyn.l3 = 1.01;   % length of third link

t0 = 0;
tF = 1.2;  %For now, force it to take exactly this much time.
x0 = [-pi/6;1e-5;pi/3];   %[q1;q2];  %initial angles   %Stable equilibrium
xF = [pi/6;1e-5;pi+2/3*pi];  %[q1;q2];  %final angles    %Inverted balance
dx0 = [1e-5;1e-5;1e-5];   %[dq1;dq2];  %initial angle rates
dxF = [1e-5;1e-5;1e-5];  %[dq1;dq2];  %final angle rates
maxTorque = 90;  % Max torque at the elbow  (GPOPS goes crazy without this)

%  * The optimal trajectory is not actually constrained by the maximum
%  torque. That being said, GPOPS goes numerically unstable if the torque
%  is not bounded. This does not seem to be a problem with the other
%  methods.

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Set up function handles                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.func.dynamics = @(t,x,u)( monkeyDynamics(t,x,u,dyn) );

problem.func.pathObj = @(t,x,u)( obj_torqueSquared(u) );  %Simple torque-squared

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%               Set up bounds on time, state, and control                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
problem.bounds.initialTime.low = t0;
problem.bounds.initialTime.upp = t0;
problem.bounds.finalTime.low = tF;
problem.bounds.finalTime.upp = tF;

% State: [q1;q2;dq1;dq2];

problem.bounds.state.low = [-2*pi; -2*pi; -2*pi; -inf(3,1)];
problem.bounds.state.upp = [ 2*pi;  2*pi;  3*pi;  inf(3,1)];

problem.bounds.initialState.low = [x0; dx0];
problem.bounds.initialState.upp = [x0; dx0];
problem.bounds.finalState.low = [xF; dxF];
problem.bounds.finalState.upp = [xF; dxF];

problem.bounds.control.low = [-maxTorque;-maxTorque];
problem.bounds.control.upp = [maxTorque;maxTorque];



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Options:                                      %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%


%%%% Run the optimization twice: once on a rough grid with a low tolerance,
%%%% and then again on a fine grid with a tight tolerance.

% method = 'trapezoid'; %  <-- this is robust, but less accurate
% method = 'direct'; %  <-- this is robust, but some numerical artifacts
% method = 'rungeKutta';  % <-- slow, gets a reasonable, but sub-optimal soln
% method = 'orthogonal';    %  <-- this usually finds bad local minimum
% method = 'gpops';      %  <-- fast, but numerical problem is maxTorque is large
method = 'hermiteSimpsonGrad';  % <-- slow, gets a reasonable, but sub-optimal soln

switch method
    case 'direct'
        problem.options(1).method = 'trapezoid';
        problem.options(1).trapezoid.nGrid = 20;
        
        problem.options(2).method = 'trapezoid';
        problem.options(2).trapezoid.nGrid = 40;
        
        problem.options(3).method = 'hermiteSimpson';
        problem.options(3).hermiteSimpson.nSegment = 20;
        
    case 'trapezoid'
        problem.options(1).method = 'trapezoid';
        problem.options(1).trapezoid.nGrid = 20;
        problem.options(2).method = 'trapezoid';
        problem.options(2).trapezoid.nGrid = 40;
        problem.options(3).method = 'trapezoid';
        problem.options(3).trapezoid.nGrid = 60;
        
    case 'rungeKutta'
        problem.options(1).method = 'rungeKutta';
        problem.options(1).defaultAccuracy = 'low';
%         
%         problem.options(2).method = 'rungeKutta';
%         problem.options(2).defaultAccuracy = 'medium';
        
    case 'orthogonal'
        problem.options(1).method = 'chebyshev';
        problem.options(1).chebyshev.nColPts = 9;
        
        problem.options(2).method = 'chebyshev';
        problem.options(2).chebyshev.nColPts = 18;
    case 'gpops'
        problem.options(1).method = 'gpops';
    
    case 'hermiteSimpsonGrad'  %hermite simpson with analytic gradients
        
        problem.options(1).method = 'hermiteSimpson'; % Select the transcription method
        problem.options(1).hermiteSimpson.nSegment = 6;  %method-specific options
        problem.options(1).nlpOpt.GradConstr = 'on';
        problem.options(1).nlpOpt.GradObj = 'on';
        problem.options(1).nlpOpt.DerivativeCheck = 'off';
        
%         problem.options(2).method = 'hermiteSimpson'; % Select the transcription method
%         problem.options(2).hermiteSimpson.nSegment = 15;  %method-specific options
%         problem.options(2).nlpOpt.GradConstr = 'on';
%         problem.options(2).nlpOpt.GradObj = 'on';
        
end
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              Create an initial guess for the trajectory                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% Start with a linear trajectory between four key frames:
% 0  --  initial configuration
% A  --  back swing
% B  --  front swing
% F  --  final configuration
%

% tA = t0 + 0.25*(tF-t0);
% xA = -x0/2;
% dxA = 2*(xF-x0)/(tF-t0);
% 
% tC = t0 + 0.5*(tF-t0);
% xC = [0;0];
% dxC = 2*(xF-x0)/(tF-t0);
% 
% tB = t0 + 0.75*(tF-t0);
% xB = -xF/2;
% dxB = 2*(xF-x0)/(tF-t0);
% 
% problem.guess.time = [t0, tA, tC, tB, tF];
% problem.guess.state = [[x0;dx0], [xA; dxA], [xC; dxC], [xB; dxB], [xF;dxF]];
% problem.guess.control = [0, 0, 0, 0, 0];

tA = t0 + 0.5*(tF-t0);
xA = [0;0;pi];
dxA = 2*(xF-x0)/(tF-t0);dxA(2)=0;


problem.guess.time = [t0, tA, tF];
problem.guess.state = [[x0;dx0], [xA; dxA], [xF;dxF]];
problem.guess.control = [[0;0], [0;0], [0;0]];

% tA = t0 + 0.5*(tF-t0);
% xA = [0;0;pi];
% dxA = 2*(xF-x0)/(tF-t0);dxA(2)=0;
% 
% 
% problem.guess.time = [t0, tF];
% problem.guess.state = [[x0;dx0],  [xF;dxF]];
% problem.guess.control = [[0;0], [0;0]];


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Solve!                                        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

soln = optimTraj(problem);

% Interpolate the solution on a uniform grid for plotting and animation:
tGrid = soln(end).grid.time;
t = linspace(tGrid(1),tGrid(end),100);
z = soln(end).interp.state(t);
u = soln(end).interp.control(t);


%% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Plot the solution                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%HINT:  type help animate to figure out how to use the keyboard to interact
%with the animation (slow motion, pause, jump forward / backward...)

% Animate the results:
A.plotFunc = @(t,z)( drawMonkey(t,z,dyn) );
A.speed = 0.5;
A.figNum = 101;
animate(t,z,A)

% Plot the results:
figure(1337); clf; plotMonkey(t,z,u(1,:),u(2,:),dyn);

% Draw a stop-action animation:
figure(1338); clf; drawStopActionMonkey(soln(end),dyn);


