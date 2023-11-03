%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Yassine Kebbati
% Date: 10/03/2021
% Control DMPC_Autonomous_Driving
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


m = 1575;
Iz = 2875;
lf = 1.2;
lr = 1.6;
Cf = 19000;
Cr = 33000;
Vx=15;
% Continuous-time model
Ac = [-(2*Cf+2*Cr)/m/Vx, 0, -Vx-(2*Cf*lf-2*Cr*lr)/m/Vx, 0;
     0, 0, 1, 0;
     -(2*Cf*lf-2*Cr*lr)/Iz/Vx, 0, -(2*Cf*lf^2+2*Cr*lr^2)/Iz/Vx, 0;
     1, Vx, 0, 0];
Bc = [2*Cf/m 0 0 0;0 0 2*Cf*lf/Iz 0]';   
Cc = [0 0 0 1; 0 1 0 0];
Dc = zeros(2,2);

h=0.1;
[Ad, Bd, Cd, Dd]=c2dm(Ac,Bc,Cc,Dc,h,'zoh');
%make the augmented state-space model
[m1, n1] = size(Cd);
[n1, n_in] = size(Bd);
A = eye(n1+m1, n1+m1);
A(1:n1, 1:n1) = Ad;
A(n1+1:n1+m1,1:n1) = Cd*Ad;
B = zeros(n1+m1, n_in);
B(1:n1,:) = Bd;
B(n1+1:n1+m1,:) = Cd*Bd;
C = zeros(m1, n1+m1);
C(:, n1+1:n1+m1) = eye(m1, m1);
D = Dc;

%generate matrices required for the predictive control cost function J
Q=C'*C;
R=0.095*eye(n_in,n_in);
a1=0.35;
a2=0.35;
N1=3; %you can increase N
N2=3;
a=[a1 a2];
N=[N1 N2];
Np=15;
[Omega,Psi]=dmpc(A,B,a,N,Np,Q,R);
[m1,n1]=size(Cd);
[n1,n_in]=size(Bd);


L_m=zeros(n_in,sum(N));
[A1,L0]=lagd(a(1),N(1));
L_m(1,1:N)=L0';
In_s=1;
for jj=2:n_in;
[Al,L0]=lagd(a(jj),N(jj));
In_s=N(jj-1)+In_s;
In_e=In_s+N(jj)-1;
L_m(jj,In_s:In_e)=L0';
end
K_mpc=L_m*(Omega\Psi);
Acl=A-B*K_mpc;


%design the state feedback control using DLQR
%[X,Y,Z]=dlqr(A_e,B_e,Q,R);
%figure
%plot(Z,'ro')
%hold on
%plot(eig(Acl),'b*')

y=zeros(m1,1);
u=zeros(n_in,1);
xm=zeros(n1,1);
N_sim=100;
r1=10*[-5*ones(1,(N_sim+10)/2) -3*ones(1,(N_sim+10)/2)];
r2=8*[5*ones(1,(N_sim+10)/2) 3*ones(1,(N_sim+10)/2)];
sp=[r1;r2];

%constraints




[M,Lzerot]=Mdu(a,N,n_in,1);
%%%%

%%%%
[u1,y1,deltau1,k]=simuuc2(xm,u,y,sp,Ad,Bd,Cd,N_sim,Omega,Psi,Lzerot);
figure 
subplot(3,1,1), plot(k,u1), title('u');
subplot(3,1,2), plot(k,y1(1,:)), title('y1'); hold on;  plot(k,r1(1:N_sim),'r'); 
subplot(3,1,3), plot(k,y1(2,:)), title('y2'); hold  on;  plot(k,r2(1:N_sim),'g');
figure 
plot(k,deltau1), title('DU');
