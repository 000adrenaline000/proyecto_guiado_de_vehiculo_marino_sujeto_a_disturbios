// OBTAINING THE LINEAR MODEL AND ALL THE HYDRODYNAMIC COEFFICIENTS FOR
// AN UNMANNED MARINE VEHICLE
m=150;  //total weigth (KG)
L=3.1;  // length of the vessel (m)
xg=0.15; //center of mass  (m)
a=0.2; // small radius of ellipse
b=0.54; //large ellipse radius
p=1024; //saltwater density
lxg=L/2+xg; //distance between xg and the propeller location
//Turning radius
rg=0.3*L; 
//Inertia Z
Ir=m*rg^2; 
Iz=m*xg^2+Ir;

alfa=0.1563; //According to the table L/2a(shortest diameter of ellipse)
U=1.0; // (m/s)
u_0=1.0; //u0

// disturbances
bd=-5; // -5 more close to experimental current

//Aggregate masses
function y=fXudot(x),y=x^0,endfunction

function y=fYrdot(x),y=x,endfunction

function y=fNrdot(x),y=x^2,endfunction
//Added mass
ma=(%pi*p*(b*a))/(2); // over 2 since only half of the ellipse is used
//Additional axial mass
    //Xudot=-m11
Xudot=-((4*alfa*p*%pi)/3)*(L/2)*(a)*(a);

Yvdot=-ma*intg(0,L,fXudot);
Yrdot=-ma*intg(0,L,fYrdot);
Yrdot=Yrdot/10; 
Nvdot=Yrdot;
Nrdot=-ma*intg(0,L,fNrdot);
Nrdot=Nrdot/10;


//Added masses
Mt=-4*alfa*p*%pi*(a*b)/2;//according to NEGMAN

Yv=U*Mt;
Yr=U*Xudot;
Nv=U*(Xudot-Yrdot);
Nr=U*(Yrdot);

Af=%pi*a^2;
Ap=L*a;
Cd=0.004*%pi*(Ap/Af)*((1+(60*((a/L)^3))+0.0025*(L/a)));
Db=-0.5*p*Af*Cd*U^2;

//
//functions
//
function y=fYvv(x),y=x^0,endfunction
function y=fNvv(x),y=x^1,endfunction
function y=fYrr(x),y=x^2,endfunction
function y=fNrr(x),y=x^2*abs(x),endfunction

Yvv=-0.5*p*(Cd*a*intg(0,L,fYvv));
Nvv=-0.5*p*(Cd*a*intg(0,L,fNvv));
Yrr=-0.5*p*(Cd*a*intg(0,L,fYrr));
Nrr=-0.5*p*(Cd*a*intg(0,L,fNrr));

// Hydrodynamic coefficients
m11=m-Yvdot;
m12=(m*xg)-Yrdot;
m21=(m*xg)-Nvdot;
m22=Iz-Nrdot;


n11=-Yv;
n12=m*u_0-Yr;
n21=(-Nv); ///8
n22=(m*xg*u_0-u_0-Nr);
// Dinamic model
// Mvdot+N(u0)v=b*Dr
M=[ m11 m12
    m21 m22];

N=[ n11 n12
    n21 n22];


k=10*9.9; // m*g of the propeller
Ydelta=k;
Ndelta=-lxg*k;
b=[ Ydelta
    Ndelta];

/////////////// STATES SPACE
    //       xdot=A*x+B1*u
// A=-M^(-1)*N               b1=M^(-1)*b
A=-inv(M)*N;
B=-inv(M)*b;
C=[1 0 ;0 1];
D=[0; 0];

a11=((m22*-n11)-(m12*-n21))/det(M);

a12=((m22*-n12)-(m12*-n22))/det(M);

a21=((m11*-n21)-(m21*-n11))/det(M);

a22=((m11*-n22)-(m21*-n12))/det(M);


b1=((m22*Ydelta)-(m12*Ndelta))/det(M);

b2=((m11*Ndelta)-(m21*Ydelta))/det(M);


An=[a11 a12
    a21 a22];
Bn=[b1
    b2];
Cn=[0 1];
Dn=[0];

sys=syslin('c',An,Bn,Cn,Dn);

// save the data
save("albaLTInomoto.sod","sys")

