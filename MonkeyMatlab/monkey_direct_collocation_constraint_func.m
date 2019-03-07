function [ c , ceq ] = monkey_direct_collocation_constraint_func(state_u)

init_angle_left_hand = -45/180*pi;
init_angle_left_shoulder = 45/180*pi;
init_angle_right_shoulder = 45/180*pi;

end_angle_left_hand = 45/180*pi;
end_angle_left_shoulder = -45/180*pi;
end_angle_right_shoulder = (45+270)/180*pi;

thetastart = [init_angle_left_hand; init_angle_left_shoulder; init_angle_right_shoulder];
thetaend = [end_angle_left_hand; end_angle_left_shoulder; end_angle_right_shoulder];

step = 100;
dt = 0.012; 

x = [thetastart;[0;0;0]];
xT = [thetaend;[0;0;0]];

state = state_u(1,1:6*step);
state = reshape(state,6,[]);
u = state_u(1,6*step+1:end);


ceq = zeros(1,6*step+12);

for i = 1:step 
    if i ~= step
        tmp_state1 = state(:,i);
        x_dot1 = monkey_dyn_func(tmp_state1, [0; u((i-1)*2+1);u((i-1)*2+2)]);
        
        tmp_state2 = 0.5*(state(:,i)+state(:,i+1));
        x_dot2 = monkey_dyn_func(tmp_state2, [0; 0.5*u((i-1)*2+1)+0.5*u(i*2+1);0.5*u((i-1)*2+2)+0.5*u(i*2+2)]);
        
        tmp_state3 = state(:,i+1);
        x_dot3 = monkey_dyn_func(tmp_state3, [0; u(i*2+1);u(i*2+2)]);
        
        delta = state(:, i) - state(:, i+1);
        delta = delta + dt/6*(x_dot1 + 4*x_dot2 + x_dot3);
        
        ceq(1,6*(i-1)+1) = delta(1);
        ceq(1,6*(i-1)+2) = delta(2);
        ceq(1,6*(i-1)+3) = delta(3);
        ceq(1,6*(i-1)+4) = delta(4);
        ceq(1,6*(i-1)+5) = delta(5);
        ceq(1,6*(i-1)+6) = delta(6);
    end
        
end

% ceq = abs(x-xT);
ceq(end-11) = abs(state(1,1)-x(1));
ceq(end-10) = abs(state(2,1)-x(2));
ceq(end- 9) = abs(state(3,1)-x(3));
ceq(end- 8) = abs(state(4,1)-x(4));
ceq(end- 7) = abs(state(5,1)-x(5));
ceq(end- 6) = abs(state(6,1)-x(6));

ceq(end- 5) = abs(state(1,end)-xT(1));
ceq(end- 4) = abs(state(2,end)-xT(2));
ceq(end- 3) = abs(state(3,end)-xT(3));
ceq(end- 2) = abs(state(4,end)-xT(4));
ceq(end- 1) = abs(state(5,end)-xT(5));
ceq(end- 0) = abs(state(6,end)-xT(6));
c = [];



end