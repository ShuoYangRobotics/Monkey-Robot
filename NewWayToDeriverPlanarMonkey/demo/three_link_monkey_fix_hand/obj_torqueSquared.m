function [dObj, dObjGrad] = obj_torqueSquared(u)
% [dObj, dObjGrad] = obj_torqueSquared(u)
%
% This function computes the torque-squared objective function and its
% gradients.
%

if nargout == 1 % numerical gradients
    
    dObj = autoGen_obj_torqueSquared(u(1,:),u(2,:));
    
else  %Analytic gradients
    
    [dObj,fz,fzi] = autoGen_obj_torqueSquared(u(1,:),u(2,:));
    dObjGrad = zeros(9,length(dObj));  % 16 = 1 + 3 + 3 + 2 = time + angle + rate + control
    dObjGrad(fzi,:) = fz;
    
end

end