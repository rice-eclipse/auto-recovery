function[xdot, y] = six_dof_parachute(x, u)

% format:
% x(1)=X (North-East-Up format)
% x(2)=Y
% x(3)=Z
% x(4)=roll (phi)
% x(5)=pitch (theta)
% x(6)=yaw (psi)
%
% x(7)=d/dt X
% x(8)=d/dt Y
% x(9)=d/dt Z
% x(10)=d/dt roll
% x(11)=d/dt pitch
% x(12)=d/dt yaw
%
% u(1)=brake
% u(2)=aileron

xdot=zeros(12,1);

%simpler variables
p = [x(1); x(2); x(3)];%X Y Z
PHI = modrange([x(4); x(5); x(6)]);%roll pitch yaw
V = [x(7); x(8); x(9)];%U V W
omega = [x(10); x(11); x(12)];%P Q R
brake = u(1);
aileron=u(2);

H=[1 tan(PHI(2))*sin(PHI(1)) tan(PHI(2))*cos(PHI(1));
    0 cos(PHI(1)) -sin(PHI(1));
    0 sin(PHI(1))/cos(PHI(2)) cos(PHI(1))/cos(PHI(2))];
Hinv = H^-1;

%cross product matrix
OMEGA = [ 0 -x(12) x(11); x(12) 0 -x(10); -x(11) x(10) 0];

%direction cosine matrix
DCM = [1 0 0; 0 cos(PHI(1)) sin(PHI(1))
    0 -sin(PHI(1)) cos(PHI(1))] *
    [cos(PHI(2)) 0 -sin(PHI(2)); 0 1 0;
    sin(PHI(2)) 0 cos(PHI(2))] *
    [cos(PHI(3)) sin(PHI(3)) 0;
    -sin(PHI(3)) cos(PHI(3)) 0; 0 0 1];

vwind = [0; 0; 0];

V = V - vwind;

%constants
g=9.81;%gravity
rho=1.2;%density of air

a=36 * 0.0254;%height of parafoil
b=84 * 0.0254;%wing span
c=24 * 0.0254;%wing chord
S=b*c;%wing area
t=5*0.0254;%thickness

Xcg = 0;%dist from confluence point to systems cg
%dist from confluence point to quarter chord
%point on parafoil along z axis
Zcg = 48 * 0.0254;

riggingangle = 10;%degrees

vc = sqrt(V’*V);%velocity
qbar = 0.5 * rho * vc*vc;%dynamic pressure
mass = 4;%mass in kg
alpha = rad2deg(atan2(V(3), V(1))) - riggingangle;

%apparent mass terms
A = 0.899 * pi() / 4 * t^2 * b;
B = 0.39 * pi() / 4 * (t^2 + 2 * deg2rad(alpha)^2) * c;
C = 0.783 * pi() / 4 * c^2 * b;

IA = 0.63 * pi() / 48 * c^2 * b^3;
IB = 0.817 * 4 / (48*pi()) * c^4*b;
IC = 1.001 * pi() / 48 * t^2 * b^3;


Kabc = [A 0 0; 0 B 0; 0 0 C];
KIabc = [IA 0 0; 0 IB 0; 0 0 IC];

[Kabc; KIabc];

% Kabc = [0 0 0; 0 0 0; 0 0 0];
% KIabc = [0 0 0; 0 0 0; 0 0 0];

%inertia matrix
J1=1.3558*[.1357 0 .0025; 0 .1506 0; .0025 0 .0203];

%J2= J1+[IA 0 0; 0 IB 0; 0 0 IC];
%J=J2 + [0 0 Xcg*Zcg; 0 sqrt(Xcg*Xcg+Zcg*Zcg) 0;
%Xcg*Zcg 0 0] * (mass+B);
J = J1 + mass * [Zcg^2 0 0; 0 Zcg^2 0; 0 0 0];

%Aerodynamic Coefficients - Iacomini & Cerimele
CLo=0.2;
CLalpha=0.0375;
CLbrake=0.377;

CDo=0.12;
CDalpha=0.0073;
CDbrakesquared=0.266;
CDbrake=0.076;

Cmo=-0.0115;
Cmalpha=-0.004;
Cmbrakesquared=0.16;
Cmbrake=0.056;

Cnr=-0.0936;%always negative for stability
Cnaileron=0.05;

%Coefficient buildup

CL=CLo + CLalpha * alpha + CLbrake*brake;%lift coefficient
CD=CDo + CDalpha * alpha
    + CDbrakesquared * brake^2 + CDbrake * brake;
Cm = Cmo + Cmalpha * alpha
    + Cmbrake*brake + Cmbrakesquared*brake^2;
    
Cn=Cnr*b/(2*vc) * rad2deg(omega(3)) + Cnaileron*aileron;
Cl = 0;

%Force & Moment buildup
F=0.5*rho*S*vc^2*(CL*[sind(riggingangle+alpha)
    0; -cosd(riggingangle+alpha)] 
    - CD*[cosd(riggingangle+alpha); 0; -sind(riggingangle+alpha)]);
M = [qbar*S*b*Cl - mass * abs(g)*Zcg*sin(PHI(1)) * cos(PHI(2));
    qbar*S*c*Cm - mass * abs(g) * Zcg * sin(PHI(2));
    qbar*S*b*Cn];%rolling, pitching & yawing moment

[alpha; V;F; vc^2];

%Force equations
%xdot(7:9)=[1/(mass+A); 1/(mass+B); 1/(mass+C)] .*
    %(F -cross(omega,V.*[mass+A;mass+B;mass+C])+mass*g*Hinv(:,3));
xdot(7:9)=(eye(3) + Kabc / mass)^-1 *
    ((F / mass) -cross(omega,V + mass^-1 * Kabc * V)+g*Hinv(:,3));
% [(eye(3) + Kabc / mass); alpha V(3) V(1)]

%Kinematic Equations
xdot(4:6)= H * omega;

%Moment Equations
% xdot(10:12)=(J^-1) * (M - OMEGA * J * ([IA; IB; IC]
%.* omega)+cross(V,V.*[mass+A;mass+B;mass+C]));
% xdot(10:12)=(J + KIabc)^-1 * (M - OMEGA * J * omega);
xdot(10:12)=(J + KIabc)^-1 *
    (M -cross(omega,KIabc * omega) -
    cross(V,Kabc * V) - OMEGA * J * omega);

    
%Navigation Equations
xdot(1:3) = [1 0 0; 0 1 0; 0 0 -1] * DCM’ * V
