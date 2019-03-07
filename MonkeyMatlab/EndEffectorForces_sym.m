function JTFtip = EndEffectorForces_sym(thetalist, Ftip, Mlist, Glist, Slist)


n = size(thetalist, 1);
JTFtip = InverseDynamics_sym(thetalist, zeros(n, 1,'sym'), zeros(n, 1,'sym'), ...
                         zeros(3, 1,'sym'), Ftip, Mlist, Glist, Slist);
end