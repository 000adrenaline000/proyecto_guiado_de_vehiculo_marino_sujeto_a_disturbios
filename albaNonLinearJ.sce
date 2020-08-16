function dotx=albaNonLinearJ(u1,u2,u3,u4,u5)
    // Load the parameters

//States variables
v=u1;      //sway
r=u2;      //yaw rate
y=u3;      //inertial position y
apsi=u4;   //yaw angle
    
deltaP=u5; //propeller angle


// external forces
f1=Yv*u_0*v+Yvv*v*abs(v)+Yrr*r*abs(r)+Yr*u_0*r-m*u_0*r+k*sin(deltaP)+bd;
f2=Nv*u_0*v+Nvv*v*abs(v)+Nrr*r*abs(r)+Nr*u_0*r-m*xg*u_0*r-lxg*k*sin(deltaP);

//
dotx=[M\[f1;f2];u_0*sin(apsi)+v*cos(apsi);r];

endfunction
