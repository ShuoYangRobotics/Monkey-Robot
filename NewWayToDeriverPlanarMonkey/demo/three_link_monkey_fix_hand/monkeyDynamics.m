function [dx, dxGrad] = monkeyDynamics(t,x,u,p)

%these code are from kelly's fivelink biped
nt = length(t);
empty = zeros(1,nt);  % For vectorization
q = x(1:3,:);
dq = x(4:6,:);
ddq = zeros(size(q));

if nargout == 1   % no gradient
    [m,mi,f,fi] = autoGen_dynSs(...
        q(1,:),q(2,:),q(3,:),...
        dq(1,:),dq(2,:),dq(3,:),...
        u(1,:),u(2,:),...
        p.m1, p.m2, p.m3, p.l1, p.l2, p.l3, p.g, empty);
    
    M = zeros(3,3);  %Mass matrix
    F = zeros(3,1);
    for i=1:nt
        M(mi) = m(:,i);
        F(fi) = f(:,i);
        ddq(:,i) = M\F;  %Numerically invert the mass matrix
    end

else  % Using analytic gradients
    [m,mi,f,fi,mz,mzi,mzd,fz,fzi,fzd] = autoGen_dynSs(...
        q(1,:),q(2,:),q(3,:),...
        dq(1,:),dq(2,:),dq(3,:),...
        u(1,:),u(2,:),...
        p.m1, p.m2, p.m3, p.l1, p.l2, p.l3, p.g, empty);
    
    M = zeros(3,3);  %Mass matrix
    F = zeros(3,1);
    nz = 9;   %Number of dimensions for gradients [t;q;dq;u]
    ddqGrad = zeros(3,nz,nt);
    Mz = zeros(mzd); 
    Fz = zeros(fzd);
    for i=1:nt
        M(mi) = m(:,i);
        F(fi) = f(:,i);
        Mz(mzi) = mz(:,i);
        Fz(fzi) = fz(:,i);
        ddq(:,i) = M\F;  %Numerically invert the mass matrix
        for j=1:nz
            ddqGrad(:,j,i) = M\( -Mz(:,:,j)*ddq(:,i) + Fz(:,:,j) );  % Derivative of a matrix inverse http://www.atmos.washington.edu/~dennis/MatrixCalculus.pdf
        end
    end
    
    dqGrad = zeros(3,nz,nt); 
    for i=1:3
       dqGrad(i,1+3+i,:) = 1; 
    end
    
    dxGrad = cat(1,dqGrad,ddqGrad);
    
end

dx = [dq;ddq];

end