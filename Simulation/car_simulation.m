function car_simulation

function ds=carddt(~,s)
%两轮之间距离，左轮转速，右轮转速
global l; global u1; global u2;

l = 1; u1 = 2; u2 = 3;
x1 = s(1); y1 = s(2); x2 = s(3); y2 = s(4);
%状态方程
dx1 = 1/l.*(y1-y2).*u1;
dy1 = 1/l.*(x2-x1).*u1;
dx2 = 1/l.*(y1-y2).*u2;
dy2 = 1/l.*(x2-x1).*u2;

ds = [dx1;dy1;dx2;dy2];
end

ODEFUN = @carddt;
s0 = [0, 0, 1, 0];%初态

step = 0.1;%步长
t_start = 0;
t_end = t_start+step;

[~,s]= ode45(ODEFUN, [t_start,t_end], s0);
% disp(size(t))
% disp(size(s))
%disp(t(size(t,1),1))

set(gcf, 'doublebuffer', 'on')%防止闪烁
figure(1)
for i=1:100
    scatter(s(size(s,1),1), s(size(s,1),2))
    hold on
    scatter(s(size(s,1),3), s(size(s,1),4))
    axis([-10 10 -10 10]); xlabel('X'); ylabel('Y')
    drawnow
    hold off

    t_start = t_end;
    t_end = t_start+step;
    s_start = [s(size(s,1),1), s(size(s,1),2), s(size(s,1),3), s(size(s,1),4)];
    
    [~,s]= ode45(ODEFUN, [t_start,t_end], s_start);
end

end
