nmues=100;
time=1:nmues;

xcen0=0;
ycen0=0;
xcen1=20;
ycen1=20;
xcen2=40;
ycen2=0;
xcen3=60;
ycen3=20;
xcen4=80;
ycen4=0;
xcen5=100;
ycen5=20;

radius=2*L;
xcir0= xcen0 + radius*cos(2*%pi*time/nmues);
ycir0= ycen0 + radius*sin(2*%pi*time/nmues);

xcir1= xcen1 + radius*cos(2*%pi*time/nmues);
ycir1= ycen1 + radius*sin(2*%pi*time/nmues);

xcir2= xcen2 + radius*cos(2*%pi*time/nmues);
ycir2= ycen2 + radius*sin(2*%pi*time/nmues);

xcir3= xcen3 + radius*cos(2*%pi*time/nmues);
ycir3= ycen3 + radius*sin(2*%pi*time/nmues);

xcir4= xcen4 + radius*cos(2*%pi*time/nmues);
ycir4= ycen4 + radius*sin(2*%pi*time/nmues);

xcir5= xcen5 + radius*cos(2*%pi*time/nmues);
ycir5= ycen5 + radius*sin(2*%pi*time/nmues);

scf(11)
plot2d(xcir0,ycir0,[12]);
plot2d(xcir1,ycir1,[12]);
plot2d(xcir2,ycir2,[12]);
plot2d(xcir3,ycir3,[12]);
plot2d(xcir4,ycir4,[12]);
plot2d(xcir5,ycir5,[12]);
xpoint=[0 20 40 60 80 100];
ypoint=[0 20 0 20 0 20];
plot2d(xpoint,ypoint,[-3],"011"," ",[-10,-10,100,30]);
yi=interpln([xpoint;ypoint],0:100);
plot2d((0:100),yi,3,"000");
plot2d(Xresul.values, Yresul.values,[2],"000")
xtitle("TRACKING ZIG ZAG");
xlabel("X position (m)", "fontsize", 5);
ylabel("Y position (m)","fontsize",5)
Leg=legend("Circulo1","Circulo2","Circulo3","Circulo4","Circulo5","Circulo6","Points of track","Reference","Usv Position")
Leg.font_size=3;
