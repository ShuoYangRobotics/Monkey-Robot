function grav = GravityForces_sym(thetalist, g, Mlist, Glist, Slist)


n = size(thetalist, 1);
grav = InverseDynamics_sym(thetalist, zeros(n, 1,'sym'), zeros(n, 1,'sym') ,g, ...
                       zeros(6, 1,'sym'), Mlist, Glist, Slist);
end