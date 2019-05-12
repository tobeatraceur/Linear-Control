function car

function ds=carddt(t,s)
%两轮之间距离，左轮转速，右轮转速
global l; global u1; global u2;

l = 1; u1 = 1; u2 = 1;
x1 = s(1); y1 = s(2); x2 = s(3); y2 = s(4);
%状态方程
dx1 = 1/l.*(y1-y2).*u1;
dy1 = 1/l.*(x2-x1).*u1;
dx2 = 1/l.*(y1-y2).*u2;
dy2 = 1/l.*(x2-x1).*u2;

ds = [dx1;dy1;dx2;dy2];
end

ODEFUN = @carddt;
s0 = [0, 0, 1, 0];
tspan = 20;
[~,s]= ode45(ODEFUN, [0,tspan], s0);

set(gcf, 'doublebuffer', 'on')%防止闪烁

figure(1)
for i =1:3*tspan
    scatter(s(i,1), s(i,2))
    hold on
    scatter(s(i,3), s(i,4))
    axis([-3 3 -3 3]); xlabel('X'); ylabel('Y');
    drawnow
    hold off
end

end