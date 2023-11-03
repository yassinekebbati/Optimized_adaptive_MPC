%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Yassine Kebbati
% Date: 10/03/2021
% Control DMPC-Autonomous_Driving
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [M,Lzerot]=Mdu(a,N,n_in,Nc)
%a and N are for the Laguerre functions
%n_in is the number of inputs
%Nc is the number of constraints
N_pa=sum(N);
M=zeros(n_in,N_pa);
M_du1=zeros(n_in,N_pa);
k0=1;
[Al,L0]=lagd(a(k0),N(k0));
M_du1(1,1:N(1))=L0';
cc=N(1);
for k0=2:n_in;
[Al,L0]=lagd(a(k0),N(k0));
M_du1(k0,cc+1:cc+N(k0))=L0';
cc=cc+N(k0);
end
Lzerot=M_du1;
M=M_du1;
for kk=2:Nc
k0=1;
[Al,L0]=lagd(a(k0),N(k0));
L=Al^(kk-1)*L0;
M_du1(1,1:N(1))=L';
cc=N(1);
for k0=2:n_in;
[Al,L0]=lagd(a(k0),N(k0));
L=Al^(kk-1)*L0;
M_du1(k0,cc+1:cc+N(k0))=L';
cc=cc+N(k0);
end
M=[M;M_du1];
end