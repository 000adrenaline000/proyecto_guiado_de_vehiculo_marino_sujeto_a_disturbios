// load the data
load("albaLTInomoto.sod","sys")

Ap=sys.A;
Bp=sys.B;
Cp=sys.C;
Dp=sys.D;

//checking controllability and observability
[i,j] = size(Ap);
// e=[B, AB, A^2 B,..., A^(n-1) B]
e = cont_mat(sys.A,sys.B);
rankC=rank(e);
if i == rankC then
    disp('Continuous System is Controllable');
end

// o=[C; CA; CA^2;...; CA^(n-1) ]
o = obsv_mat(sys.A, sys.C);
rankO=rank(o);
if j == rankO then
    disp('Continuous System is Observable');
end

//tranM=ss2tf(sys); // Matriz de transferencia
//disp('Matriz de Transferencia',tranM);

//tfc11 = tranM(1,1);
//tfc22 = tranM(2,2); 

/* Plot singular values of LTI the model */
tr = trzeros(sys)
w = logspace(-3,3);
sv = svplot(sys,w);
scf(1);
plot2d("ln", w, 20*log(sv')/log(10))
xgrid(12)
xtitle("Singular values plot","Frequency (rad/s)", "Amplitude (dB)");

//Obtenciion de los polors  zeros del modelo de software
scf(2);
plzr(sys);
///////////////////////////////////---------------------------

//Augment Plant with Integrators at Plant Input
[ns,nc]=size(Bp); //ns= number of inputs; nc=number of controls
Ai=[Ap             Bp;
    0*ones(nc,ns) 0*ones(nc,nc)];

Bi=[0*ones(ns,nc); eye(nc)];
    
Ci=[Cp 0*ones(nc,nc)];

Di=0*ones(nc,nc);

sysi=syslin('c',Ai,Bi,Ci,Di);

I=eye(nc);

/* Plot singular values  */
tri = trzeros(sysi)
w = logspace(-3,3);
svi = svplot(sysi,w);
scf(3);
plot2d("ln", w, 20*log(svi')/log(10))
xgrid(12)
xtitle("Design Plant Singular Values","Frequency (rad/s)", "Amplitude (dB)");

//Obtenciion de los polors  zeros del modelo de software
scf(4);
plzr(sysi);

//lqr controller calculation
//We use the ricatti equation for calculate de gain of the lqr controller
//for this we have  A'*X+X*A-X*B*X+C=0 for function X=riccati(A,B,C,'c','eigen')
C=0.7*Ci'*Ci;        //State Weighting Matrix
rho=1e-1;       //Cheap control recovery parameter 
                //The smaller the parameter, the better the recovery.
R = rho*eye(nc);//Control Weigthing Matrix


//now we calculate B
B=Bi*inv(R)*Bi';

A=Ai;

//Solv the ricatti equation
X=riccati(A,B,C,'c','eigen');

//the value of the gain G
G=inv(R)*Bi'*X; //<--this value is important mtfk

//[G1, X1]=lqr(sysi,C,R);

//Design of Target Loop Singular Values Using Kalman Filter

ll= inv(Cp*inv(-Ap)*Bp+Dp);      //Choose ll and lh to match singular values at all frequencies
lh = -inv(Ap)*Bp*ll;

Lp=[lh,
   ll];                         //ll, lh - for low and high frequency loop shaping
   
pnint = eye(nc,nc)      // Process Noise Intensity Matrix
mu = 0.1;           // Measurement Noise Intesity; Used to adjust Kalman Filter Bandwidth
                     //Small mu - expensive sensor   - large bandwidth
                     //Large mu - inexpensive sensor - small bandwidth
THETA = mu*eye(nc,nc)   // Measurement Noise Intensity Matrix 

//computing H
//We use the ricatti equation for calculate de gain H
//for this we have  Ah'*Xh+Xh*Ah-Xh*Bh*Xh+Ch=0 for function X=riccati(Ah,Bh,Ch,'c','eigen')

Ch=Lp*Lp';
Ah=Ai';
//calculating Bh
Bh=Ci'*inv(THETA)*Ci;

//Calculate de solution
Xh=riccati(Ah,Bh,Ch,'c','eigen');

//The gain H
H=(inv(THETA)*Ci*Xh)';

sysh = syslin('c',Ai,H,Ci,Di);

/* Plot singular values*/
trh = trzeros(sysh)
w = logspace(-3,3);
svh = svplot(sysh,w);
scf(5);
plot2d("ln", w, 20*log(svh')/log(10))
xgrid(12)
xtitle("Target Loop (G_{KF}) Singular Values", "Amplitude (dB)");

//--------------------------------------
//Compensator Analysis
Ak = [ Ai-Bi*G-H*Ci  0*ones(ns+nc,nc)
       G          0*ones(nc,nc) ]

Bk = [ H
       0*ones(nc,nc) ]

Ck = [0*ones(nc, ns+nc) eye(nc,nc) ]

Dk = 0*ones(nc,nc);

sysk=syslin('c',Ak,Bk,Ck,Dk);

/* Plot singular values  */
trk = trzeros(sysk)
w = logspace(-3,3);
svk = svplot(sysk,w);
scf(6);
plot2d("ln", w, 20*log(svk')/log(10))
xgrid(12)
xtitle("Compensator Singular Values","Frequency (rad/s)", "Amplitude (dB)");


//----------------------------------------
//Analysis in open loop
Aol = [ Ap                     Bp*Ck
       0*ones(ns+nc+nc,ns)    Ak    ]

Bol = [ 0*ones(ns,nc)
       Bk ]
    
Col = [ Cp  0*ones(nc,ns+nc+nc) ]

Dol = 0*ones(nc,nc);

sysol = syslin('c',Aol,Bol,Col,Dol);

/* Plot singular values of LTI the model */
trol = trzeros(sysol)
w = logspace(-3,3);
svol = svplot(sysol,w);
scf(7);
plot2d("ln", w, 20*log(svol')/log(10))
plot2d("ln", w, 20*log(svh')/log(10))
xgrid(12)
xtitle("Singular values open loop","Frequency (rad/s)", "Amplitude (dB)");

//Obtenciion de los polors  zeros del modelo de software
scf(8);
plzr(sysol);



//----------------------------------------
//Response in closed loop
syscl = syslin('c',Aol-Bol*Col, Bol, Col, 0*eye(nc,nc));

//Obtenciion de los polors  zeros del modelo de software
scf(9);
plzr(syscl);

//----------Response to step
t=[0:0.1:30];
//input defined by a time function
deff('u=timefun(t)','u=1') //*%pi/180
scf(10);
plot2d(t',(csim(timefun,t,syscl))')
xtitle("Response of the model","Amplitud", "t(s)");
