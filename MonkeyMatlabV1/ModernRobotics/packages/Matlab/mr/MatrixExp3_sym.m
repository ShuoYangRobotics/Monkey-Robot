function  R = MatrixExp3_sym(so3mat)


omgtheta = so3ToVec(so3mat);

[omghat, theta] = AxisAng3(omgtheta);
omgmat = so3mat / theta;
R = eye(3) + sin(theta) * omgmat + (1 - cos(theta)) * omgmat * omgmat;

end