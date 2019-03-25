function M = MassMatrix_sym(thetalist, Mlist, Glist, Slist)


n = size(thetalist, 1);
M = zeros(n,'sym');
for i = 1: n
   ddthetalist = zeros(n, 1);
   ddthetalist(i) = 1;
   M(:, i) = InverseDynamics_sym(thetalist, zeros(n, 1,'sym'), ddthetalist, ...
                             zeros(3, 1,'sym'), zeros(6, 1,'sym'),Mlist, ...
                             Glist, Slist);
end
end