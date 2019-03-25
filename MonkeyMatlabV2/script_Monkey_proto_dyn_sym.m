syms th1 th2 th3 dth1 dth2 dth3 real
syms tau1 tau2 real

% init dynamics
script_Monkey_proto_basic;

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

b = (action - simplify(c) ...
                 - simplify(grav) ...
                 - simplify(EndEffectorForces_sym(thetalist, Ftip, Mlist, Glist, ...
                                     Slist)));
b = simplify(b);  
M = simplify(M);
M_inv = simplify(inv(M));

ddthetalist = M_inv*b;

ddthetalist = simplify(ddthetalist);
                                 
A = jacobian(ddthetalist,[th1 th2 th3 dth1 dth2 dth3]);   

B = jacobian(ddthetalist,[tau1 tau2]); 

for i = 1:3
    for j = 1:6
        i
        j
        A(i,j) = simplify(A(i,j));
    end
end

for i = 1:3
    for j = 1:2
        i
        j
        B(i,j) = simplify(B(i,j));
    end
end

matlabFunction(A, 'Vars' ,[th1 th2 th3 dth1 dth2 dth3 tau1 tau2],'File','A_func','Comments','Version: 1.1');
matlabFunction(B, 'Vars' ,[th1 th2 th3 dth1 dth2 dth3 tau1 tau2],'File','B_func','Comments','Version: 1.1');