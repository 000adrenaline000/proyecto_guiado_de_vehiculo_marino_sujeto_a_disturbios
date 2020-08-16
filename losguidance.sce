function xylos=losguidance(u1,u2)
x=u1;
y=u2;
if(x>=5 & y>=5) then
    xlos=10
    ylos=10
else
    xlos=5
    ylos=5;
end
xylos=[xlos;ylos];

endfunction
