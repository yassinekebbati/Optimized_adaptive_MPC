%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Yassine Kebbati
% Date: 10/03/2021
% Control DMPC-Autonomous_Driving
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




% generating the data matrices in the design of discrete-time model predictive 
% control system where the cost function is expressed as $J= eta^T*E*eta +2eta^T*H*x(ki)*
% with Laguerre functions for a multi-input system with weight matrix Q on state variables and R on the input variables

function [E,H]=dmpc(A_e,B_e,a,N,Np,Q,R);
%A_e;B_e define the extended state-space model when
% integrator is used
%they can also be other forms of state-space models
% a contains the Laguerre pole locations for each input
%N the number of terms for each input
%Np prediction horizon
%Q weight on the state variables
%R weight on the input variables assumed to be diagonal.
% The cost function is J= eta ^T E eta +2 eta ^T H x(k_i)
[n,n_in]=size(B_e);
N_pa=sum(N); %the dimension of eta
E=zeros(N_pa,N_pa);
H=zeros(N_pa,n);
R_para=zeros(N_pa,N_pa);
n0=1;
ne=N(1);
for i=1:n_in-1;
R_para(n0:ne,n0:ne)= R(i,i)*eye(N(i),N(i));
n0=n0+N(i);
ne=ne+N(i+1);
end
R_para(n0:N_pa,n0:N_pa)=R(n_in,n_in)*eye(N(n_in),N(n_in));

% The part below prepares the initial conditions for the convolution sum
% and calculates the case i = 1. Each input takes a position in the convolution sum.

S_in=zeros(n,N_pa);
[Al,L0]=lagd(a(1),N(1));
S_in(:,1:N(1))=B_e(:,1)*L0';
In_s=1;
for jj=2:n_in;
[Al,L0]=lagd(a(jj),N(jj));
In_s=N(jj-1)+In_s;
In_e=In_s+N(jj)-1;
S_in(:,In_s:In_e)=B_e(:,jj)*L0';
end
S_sum=S_in;
phi=S_in;
E=(phi)'*Q*(phi);
H=phi'*Q*A_e;

% There are two iterations in the program below. Iteration i is with respect
% to the prediction horizon; and iteration kk is with respect to the number of
% input variables. Upon obtaining the convolution sum, the E and H matrices
% are the sum of the iterations with respect to i.

for i=2:Np;
Eae=A_e^i;
%calculate the finite sum S for each input
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%For each sample i
%%%%%%%%%%%%calculate input number 1
%specify the L0 and state matrix Al
%associated with the first input
[Al,L0]=lagd(a(1),N(1));
% Laguerre function associated with input number 1
S_sum(:,1:N(1))=A_e*S_sum(:,1:N(1))+...
S_in(:,1:N(1))*(Al^(i-1))';
%%move on to input number 2 and so on
In_s=1;
for kk=2:n_in;
[Al,L0]=lagd(a(kk),N(kk));
In_s=N(kk-1)+In_s;
In_e=In_s+N(kk)-1;
S_sum(:,In_s:In_e)=A_e*S_sum(:,In_s:In_e)+...
S_in(:,In_s:In_e)*(Al^(i-1))';
end
phi=S_sum;
E=E+phi'*Q*phi;
H=H+phi'*Q*Eae;
end
E=E+R_para;


% This program generates the predictive control cost function for a system
% with an arbitrary number of inputs, states and outputs



