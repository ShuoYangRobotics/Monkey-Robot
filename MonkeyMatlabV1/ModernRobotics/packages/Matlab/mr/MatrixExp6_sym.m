function T = MatrixExp6_sym(se3mat)

omgtheta = so3ToVec(se3mat(1: 3, 1: 3));

[omghat, theta] = AxisAng3(omgtheta);
omgmat = se3mat(1: 3, 1: 3) / theta; 
T = [MatrixExp3_sym(se3mat(1: 3, 1: 3)), ...
     (eye(3) * theta + (1 - cos(theta)) * omgmat ...
      + (theta - sin(theta)) * omgmat * omgmat) ...
        * se3mat(1: 3, 4) / theta;
     0, 0, 0, 1];

end