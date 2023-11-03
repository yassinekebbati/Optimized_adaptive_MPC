%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Yassine Kebbati
% Date: 10/03/2021
% Control DMPC_Autonomous_Driving
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%produce a multivariable closed-loop simulation for predictive control systems without constraints

function [u1,y1,deltau1,k] = simuuc2(xm,u,y,sp,Ap,Bp,Cp,N_sim,Omega,Psi,Lzerot)
%closed-loop simulation without constraints
[m1,n1]=size(Cp);
[n1,n_in]=size(Bp);
Xf=[xm;y-sp(:,1)];
for kk=1:N_sim;
eta=-(Omega\Psi)*Xf;
deltau=Lzerot*eta;
u=u+deltau;
deltau1(:,kk)=deltau;
u1(1:n_in,kk)=u;
y1(1:m1,kk)=y;


%%%%
%plant simulation
%%%%%%
xm_old=xm;
xm=Ap*xm+Bp*u; % calculate xm(k+1)
y=Cp*xm; %calculate y(k+1)
%updating feedback state variable Xf
Xf=[xm-xm_old;y-sp(:,kk+1)];
end
k=0:(N_sim-1);

