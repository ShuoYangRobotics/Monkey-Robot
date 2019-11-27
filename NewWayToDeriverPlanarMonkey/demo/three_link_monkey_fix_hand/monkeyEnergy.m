function [E,U,T] = monkeyEnergy(z,p)


q1 = z(1,:);
q2 = z(2,:);
q3 = z(3,:);
dq1 = z(4,:);
dq2 = z(5,:);
dq3 = z(6,:);

[U,T] = autoGen_monkeyEnergy(q1,q2,q3,dq1,dq2,dq3,p.m1,p.m2,p.m3,p.g,p.l1,p.l2,p.l3);

E = U+T;

end