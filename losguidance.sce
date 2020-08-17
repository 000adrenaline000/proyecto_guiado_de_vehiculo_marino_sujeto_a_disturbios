function xylos=losguidance(u1,u2)
global point activator xd yd
x=u1;
y=u2;
if ((((xd-x)^2) + ((yd-y)^2)) <= ((L)^2)) then
    select point,
        case 1 then 
            xd = 20;
            yd = 20;
            activator = 1;
        case 2 then 
            xd = 40;
            yd = 0;
            activator = 1;
        case 3 then
            xd = 60;
            yd = 20;
            activator = 1;
        case 4 then
            xd = 80;
            yd = 0;
            activator = 1;
        else
            xd = 100;
            yd = 20;
            activator = 1;
    end
elseif (activator == 1) then
    point = point + 1 ;
    activator = 0;
else
    xd=xd;
    yd=yd;
end
xlos = xd;
ylos = yd;
xylos=[xlos;ylos];
endfunction
