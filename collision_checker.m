function f = collision_checker(p1, p2, env)
delx = p2(1)-p1(1);
dely = p2(2)-p1(2);

%bresenham's algorithm
steps = max([abs(delx), abs(dely)]);

Xinc = delx/steps;
Yinc = dely/steps;


X = p1(1);
Y = p1(2);
f = false;
for i=0:steps 
    if env(round(Y),round(X))<255
   % if env(round(X),round(Y))<255
        f = true;
        return;
    end
    X = (X + Xinc);
    Y = (Y + Yinc);    
end