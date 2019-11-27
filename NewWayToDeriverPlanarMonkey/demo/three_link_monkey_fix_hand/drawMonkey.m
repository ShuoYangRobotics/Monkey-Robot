function drawMonkey(t,z,p)

clf; hold on;

length = p.l1+p.l2;
axis equal; axis(length*[-1,1,-1,1]); axis off;

[p1,p1e,p2,p2e,p3,p3e,v1,v2,v3] = monkeyKinematics(z,p);
pos = [[0;0],p1,p1e,p2,p2e];
pos2 = [p1e,p3,p3e];

plot(0,0,'ks','MarkerSize',25,'LineWidth',4)
plot(pos(1,:),pos(2,:),'Color',[0.1, 0.8, 0.1],'LineWidth',4)
plot(pos(1,:),pos(2,:),'k.','MarkerSize',50)
plot(pos2(1,:),pos2(2,:),'Color',[0.1, 0.8, 0.1],'LineWidth',4)
plot(pos2(1,:),pos2(2,:),'k.','MarkerSize',50)

title(sprintf('Monkey Animation,  t = %6.4f', t));

drawnow; pause(0.001); 

end