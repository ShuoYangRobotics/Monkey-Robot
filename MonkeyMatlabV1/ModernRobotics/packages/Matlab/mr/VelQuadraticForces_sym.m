function c = VelQuadraticForces_sym(thetalist, dthetalist, Mlist, Glist, Slist)


c = InverseDynamics_sym(thetalist, dthetalist, ...
                    zeros(size(thetalist, 1), 1,'sym'), zeros(3,1,'sym'), ...
                    zeros(6,1,'sym'), Mlist, Glist, Slist);
end